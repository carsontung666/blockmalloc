#pragma once
/*
blockmalloc: 固定大小内存块分配器（block pool）。C11，无外部依赖。

用途：管理固定 block_size 的块数组，线程/进程安全（原子自旋锁）。
boxmalloc 依赖本库管理 box_head_t 节点。

布局：meta 区（blocks_meta_t）+ block 区（连续 block 数组，每 block = block_head + data）。
空闲链表：单向链，free_next_id 哨兵 -1。block head 自动选 2/4/8 字节节省内存。
*/

/*
blockmalloc: 固定大小内存块分配器（block pool）。C11，无外部依赖。

用途：管理固定 block_size 的块数组，线程/进程安全（原子自旋锁）。
boxmalloc 依赖本库管理 box_head_t 节点。

布局：meta 区（blocks_meta_t）+ block 区（连续 block 数组，每 block = block_head + data）。
空闲链表：单向链，free_next_id 哨兵 -1。block head 自动选 2/4/8 字节节省内存。

用法：
  - 链接 .so:   #include "blockmalloc/blockmalloc.h" + link -lblockmalloc
  - 单头文件:   在一个 .c 中 #define BLOCKMALLOC_IMPLEMENTATION，然后 #include "blockmalloc/blockmalloc.h"
                其余 .c 直接 #include "blockmalloc/blockmalloc.h"
*/

#ifndef blockmalloc_H
#define blockmalloc_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint64_t total_size;
    uint64_t block_size;
    uint8_t  sizeof_block_head:4;
    uint64_t malloc_blocks:60;
    uint64_t used_blocks;
    int64_t  free_next_id;   // 首个空闲块 ID，-1=无空闲
    int64_t  lock;            // 原子自旋锁
} blocks_meta_t;

int     blocks_init(blocks_meta_t *meta, uint64_t total_size, uint64_t block_size);
int64_t blocks_alloc(blocks_meta_t *meta, void *block_start);
void    blocks_free(blocks_meta_t *meta, void *block_start, uint64_t block_id);

// 辅助偏移计算（实现 TU 内联；无外部消费者，不导出符号）
#ifdef BLOCKMALLOC_IMPLEMENTATION
#define BLOCKS_INLINE static inline
#else
#define BLOCKS_INLINE
#endif
BLOCKS_INLINE int64_t block_offset(const blocks_meta_t *meta, uint64_t block_id);
BLOCKS_INLINE int64_t blockdata_offset(const blocks_meta_t *meta, uint64_t block_id);
int64_t blockid_byblockoffset(const blocks_meta_t *meta, uint64_t block_offset);
int64_t blockid_bydataoffset(const blocks_meta_t *meta, uint64_t data_offset);

#endif

#ifdef BLOCKMALLOC_IMPLEMENTATION

// ============================================================
// implementation (from src/blockmalloc.c + src/logutil.h)
// ============================================================

// --- logutil ---
#ifdef ENABLE_LOG
    #ifdef PLATFORM_LOG
        #define LOG(fmt, ...) PLATFORM_LOG("[blockmalloc] " fmt, ##__VA_ARGS__)
    #else
        #define LOG(fmt, ...) ((void)0)
    #endif
#else
    #define LOG(fmt, ...) ((void)0)
#endif

// --- spinlock ---
#if defined(_MSC_VER)
#include <windows.h>
#include <intrin.h>
#else
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#if !defined(_POSIX_C_SOURCE) && !defined(_GNU_SOURCE) && !defined(_DEFAULT_SOURCE)
int kill(pid_t pid, int sig);
#endif
#endif

/* architecture-specific CPU-relax / pause */
#if defined(__x86_64__) || defined(__i386__)
static inline void cpu_relax(void) { __asm__ __volatile__("pause" ::: "memory"); }
#elif defined(__aarch64__) || defined(__arm__)
static inline void cpu_relax(void) { __asm__ __volatile__("yield" ::: "memory"); }
#elif defined(_MSC_VER)
static inline void cpu_relax(void) { YieldProcessor(); }
#else
static inline void cpu_relax(void) { (void)0; }
#endif

#ifdef _MSC_VER
static inline int64_t spin_self_pid(void)
{
    int64_t p = (int64_t)GetCurrentProcessId();
    return p > 0 ? p : 1;
}
static inline int spin_lock_ex(int64_t *lock)
{
    int64_t me = spin_self_pid();
    while (InterlockedCompareExchange64((volatile LONG64 *)lock, (LONG64)me, 0) != 0)
    {
        cpu_relax();
    }
    return 0;
}
static inline void spin_lock(int64_t *lock)
{
    (void)spin_lock_ex(lock);
}
static inline void spin_unlock(int64_t *lock)
{
    InterlockedExchange64((volatile LONG64 *)lock, 0);
}
#else
static _Thread_local int64_t bm_tls_pid;
static pthread_once_t bm_atfork_once = PTHREAD_ONCE_INIT;

static void bm_reset_tls_pid(void)
{
    bm_tls_pid = 0;
}

static void bm_install_atfork(void)
{
    (void)pthread_atfork(NULL, NULL, bm_reset_tls_pid);
}

static inline int64_t spin_self_pid(void)
{
    int64_t p = bm_tls_pid;
    if (__builtin_expect(p <= 0, 0))
    {
        (void)pthread_once(&bm_atfork_once, bm_install_atfork);
        p = (int64_t)getpid();
        if (p <= 0)
            p = 1;
        bm_tls_pid = p;
    }
    return p;
}

static inline bool spin_pid_dead(int64_t owner)
{
    if (owner <= 0)
        return false;
    if (kill((pid_t)owner, 0) == -1 && errno == ESRCH)
        return true;
#if defined(__linux__)
    char path[64];
    int n = snprintf(path, sizeof(path), "/proc/%lld/stat", (long long)owner);
    if (n > 0 && (size_t)n < sizeof(path))
    {
#ifdef O_CLOEXEC
        int fd = open(path, O_RDONLY | O_CLOEXEC);
#else
        int fd = open(path, O_RDONLY);
#endif
        if (fd < 0)
            return errno == ENOENT;
        char buf[512];
        ssize_t r = read(fd, buf, sizeof(buf) - 1);
        close(fd);
        if (r > 0)
        {
            buf[r] = '\0';
            char *rp = strrchr(buf, ')');
            if (rp)
            {
                char *s = rp + 1;
                while (*s == ' ')
                    s++;
                if (*s == 'Z' || *s == 'X')
                    return true;
            }
        }
    }
#endif
    return false;
}

/* Returns 1 if taken over from a dead/zombie owner, 0 on normal acquire. */
static inline int spin_lock_ex(int64_t *lock)
{
    int64_t me = spin_self_pid();
    for (;;)
    {
        int64_t expected = 0;
        if (__atomic_compare_exchange_n(lock, &expected, me, false,
                                        __ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
            return 0;
        for (int spin = 0; spin < 4096; spin++)
        {
            cpu_relax();
            if (__atomic_load_n(lock, __ATOMIC_RELAXED) == 0)
                break;
        }
        int64_t owner = __atomic_load_n(lock, __ATOMIC_ACQUIRE);
        if (owner > 0 && owner != me && spin_pid_dead(owner))
        {
            if (__atomic_compare_exchange_n(lock, &owner, me, false,
                                            __ATOMIC_ACQ_REL, __ATOMIC_RELAXED))
                return 1;
        }
    }
}

static inline void spin_lock(int64_t *lock)
{
    (void)spin_lock_ex(lock);
}

static inline void spin_unlock(int64_t *lock)
{
    __atomic_store_n(lock, 0, __ATOMIC_RELEASE);
}
#endif

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif

typedef struct
{
    uint8_t used : 1;
    uint8_t has_next_free : 1;
    uint16_t next_free_id : 14;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
block16_t;

typedef struct
{
    uint8_t used : 1;
    uint8_t has_next_free : 1;
    uint32_t next_free_id : 30;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
block32_t;

typedef struct
{
    uint8_t used : 1;
    uint8_t has_next_free : 1;
    uint64_t next_free_id : 62;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
block64_t;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

static inline void setblock_t(const blocks_meta_t *meta, void *block_ptr, uint8_t used, int64_t free_next_id)
{
    switch (meta->sizeof_block_head)
    {
    case 2:
    {
        block16_t *block = (block16_t *)block_ptr;
        block->used = used;
        block->has_next_free = (free_next_id != -1) ? 1 : 0;
        block->next_free_id = (free_next_id != -1) ? (uint16_t)free_next_id : 0;
        break;
    }
    case 4:
    {
        block32_t *block = (block32_t *)block_ptr;
       block->used = used;
        block->has_next_free = (free_next_id != -1) ? 1 : 0;
        block->next_free_id = (free_next_id != -1) ? (uint32_t)free_next_id : 0;
        break;
    }
    case 8:
    {
        block64_t *block = (block64_t *)block_ptr;
       block->used = used;
        block->has_next_free = (free_next_id != -1) ? 1 : 0;
        block->next_free_id = (free_next_id != -1) ? (uint64_t)free_next_id : 0;
        break;
    }
    default:
        LOG("[ERROR] Invalid sizeof_block_head: %d, returning default block", meta->sizeof_block_head);
        // result 已初始化为 0，无需额外操作
        break;
    }
}

static inline block64_t getblock_t(const blocks_meta_t *meta, void *block_ptr)
{
    switch (meta->sizeof_block_head)
    {
    case 2:
    {
        block16_t *block = (block16_t *)block_ptr;
        return (block64_t){
            .used = block->used,
            .has_next_free = block->has_next_free,
            .next_free_id = block->next_free_id,
        };
    }
    case 4:
    {
        block32_t *block = (block32_t *)block_ptr;
        return (block64_t){
            .used = block->used,
            .has_next_free = block->has_next_free,
            .next_free_id = block->next_free_id,
        };
    }
    case 8:
    {
        block64_t *block = (block64_t *)block_ptr;
        return (block64_t){
            .used = block->used,
            .has_next_free = block->has_next_free,
            .next_free_id = block->next_free_id,
        };
    }
    default:
        // 处理无效 sizeof_block_head（例如，数据损坏）
        LOG("[ERROR] Invalid sizeof_block_head: %d, returning default block", meta->sizeof_block_head);
        // result 已初始化为 0，无需额外操作
        return (block64_t){0};
    }
}

#define BLOCK_SIZE(meta) (meta->sizeof_block_head + meta->block_size)

int blocks_init(blocks_meta_t *meta, const uint64_t total_size, const uint64_t block_size)
{
    if (meta == NULL)
    {
        LOG("[ERROR] meta is NULL");
        return -1;
    }
    *meta = (blocks_meta_t){
        .total_size = total_size,
        .block_size = block_size,
        .malloc_blocks = 0,
        .used_blocks = 0,
        .free_next_id = -1, // 初始化为 -1，表示没有空闲块，需要新增分配
        .lock = 0,          // 初始化锁为未锁定状态
    };
    uint64_t min_header_size = 2; // 假设最小 2 字节 (int16_t)
    uint64_t max_blocks = meta->total_size / (min_header_size + meta->block_size);
    if (max_blocks <= (32767ULL / 4ULL))
        meta->sizeof_block_head = 2; // int16_t
    else if (max_blocks <= (2147483647ULL / 4ULL))
        meta->sizeof_block_head = 4; // int32_t
    else
        meta->sizeof_block_head = 8; // int64_t
    return 0;
}

BLOCKS_INLINE int64_t block_offset(const blocks_meta_t *meta, const uint64_t block_id)
{
    int64_t offset = block_id * BLOCK_SIZE(meta);
    return offset;
}
BLOCKS_INLINE int64_t blockdata_offset(const blocks_meta_t *meta, const uint64_t block_id)
{
    int64_t offset = block_offset(meta, block_id);
    return offset + meta->sizeof_block_head;
}

int64_t blockid_byblockoffset(const blocks_meta_t *meta, const uint64_t block_offset)
{
    uint64_t block_size = BLOCK_SIZE(meta);
    if (block_offset % block_size != 0)
    {
        // 计算建议的正确偏移量（向下取整到最近的块边界）
        uint64_t suggested_offset = block_offset - (block_offset % block_size);
        LOG("[ERROR] block_offset %zu is not aligned with BLOCK_SIZE %zu, suggested offset: %zu",
            block_offset, block_size, suggested_offset);
        return -1;
    }
    uint64_t block_id = block_offset / block_size;
    return block_id;
}

int64_t blockid_bydataoffset(const blocks_meta_t *meta, const uint64_t data_offset)
{
    return blockid_byblockoffset(meta, data_offset - meta->sizeof_block_head);
}

static inline void bm_store_malloc_blocks(blocks_meta_t *meta, uint64_t n)
{
#ifdef _MSC_VER
    meta->malloc_blocks = n;
#else
    /* sizeof_block_head:4 and malloc_blocks:60 share one 64-bit word. */
    uint8_t head = meta->sizeof_block_head;
    blocks_meta_t tmp;
    memset(&tmp, 0, sizeof(tmp));
    tmp.sizeof_block_head = head;
    tmp.malloc_blocks = n;
    uint64_t word;
    memcpy(&word, (const char *)&tmp + offsetof(blocks_meta_t, block_size) + sizeof(uint64_t), sizeof(word));
    uint64_t *dst = (uint64_t *)((char *)meta + offsetof(blocks_meta_t, block_size) + sizeof(uint64_t));
    __atomic_store_n(dst, word, __ATOMIC_RELEASE);
#endif
}

/* Rebuild free_next_id and used_blocks from per-block headers under meta->lock. */
static inline void blocks_repair_pool(blocks_meta_t *meta, void *block_start)
{
    uint64_t stride = BLOCK_SIZE(meta);
    if (stride == 0)
        return;
    uint64_t max_b = meta->total_size / stride;
    uint64_t mb = meta->malloc_blocks;
    if (mb > max_b)
    {
        mb = max_b;
        bm_store_malloc_blocks(meta, mb);
    }
    int64_t head_free = -1;
    uint64_t used = 0;
    for (uint64_t id = mb; id > 0; id--)
    {
        uint64_t bid = id - 1;
        uint8_t *ptr = (uint8_t *)block_start + block_offset(meta, bid);
        block64_t blk = getblock_t(meta, ptr);
        if (blk.used == 1)
        {
            used++;
        }
        else
        {
            setblock_t(meta, ptr, 0, head_free);
            head_free = (int64_t)bid;
        }
    }
#ifdef _MSC_VER
    meta->free_next_id = head_free;
    meta->used_blocks = used;
#else
    __atomic_store_n(&meta->free_next_id, head_free, __ATOMIC_RELEASE);
    __atomic_store_n(&meta->used_blocks, used, __ATOMIC_RELEASE);
#endif
}

int64_t blocks_alloc(blocks_meta_t *meta, void *block_start)
{
    if (meta == NULL || block_start == NULL)
    {
        LOG("[ERROR] meta or block_start is NULL");
        return -1;
    }
    if (spin_lock_ex(&meta->lock))
        blocks_repair_pool(meta, block_start);
    int64_t head_free = meta->free_next_id;
    if (head_free == -1)
    {
        uint64_t cur_malloc = meta->malloc_blocks;
        uint64_t totalused_size = block_offset(meta, cur_malloc);
        if (totalused_size + BLOCK_SIZE(meta) > meta->total_size)
        {
            LOG("[ERROR] out of memory,%zu(used_size)= %zu(malloc_blocks)*(sizeof(block_head)=%d)+%zu(block_size)),when total_size %zu",
                totalused_size, cur_malloc, (uint8_t)(meta->sizeof_block_head), meta->block_size, meta->total_size);
            spin_unlock(&meta->lock);
            return -1;
        }
        else
        {
            uint64_t block_id = cur_malloc;
            int64_t b_offset = block_offset(meta, block_id);
            setblock_t(meta, (uint8_t *)block_start + b_offset, 1, -1);
            bm_store_malloc_blocks(meta, cur_malloc + 1);
            meta->used_blocks++;
            LOG("[INFO] append block %zu,blocks usage: %zu/%zu", block_id, (uint64_t)(meta->used_blocks), (uint64_t)(meta->malloc_blocks));
            spin_unlock(&meta->lock);
            return block_id;
        }
    }
    else
    {
        uint64_t free_id = (uint64_t)head_free;
        int64_t b_offset = block_offset(meta, free_id);
        block64_t free_block = getblock_t(meta, (uint8_t *)block_start + b_offset);
        int64_t next_id = free_block.has_next_free ? (int64_t)free_block.next_free_id : -1;
        setblock_t(meta, (uint8_t *)block_start + b_offset, 1, -1);
#ifdef _MSC_VER
        meta->free_next_id = next_id;
#else
        __atomic_store_n(&meta->free_next_id, next_id, __ATOMIC_RELEASE);
#endif
        meta->used_blocks++;

        LOG("[INFO] reusing block %zu, blocks usage: %zu/%zu",
            free_id, (uint64_t)(meta->used_blocks), (uint64_t)(meta->malloc_blocks));
        spin_unlock(&meta->lock);
        return free_id;
    }
}

void blocks_free(blocks_meta_t *meta, void *block_start, const uint64_t block_id)
{
    if (meta == NULL || block_start == NULL)
    {
        LOG("[ERROR] meta or block_start must not NULL");
        return;
    }
    if (spin_lock_ex(&meta->lock))
        blocks_repair_pool(meta, block_start);
    if (block_id >= meta->malloc_blocks)
    {
        LOG("[ERROR] block id %zu out of range", block_id);
        spin_unlock(&(meta->lock));
        return;
    }
    int64_t b_offset = block_offset(meta, block_id);
    block64_t free_block = getblock_t(meta, (uint8_t *)block_start + b_offset);

    switch (free_block.used)
    {
    case 0:
        LOG("[WARN] block id %zu already free", block_id);
        break;
    case 1:
        setblock_t(meta, (uint8_t *)block_start + b_offset, 0, meta->free_next_id);
#ifdef _MSC_VER
        meta->free_next_id = (int64_t)block_id;
#else
        __atomic_store_n(&meta->free_next_id, (int64_t)block_id, __ATOMIC_RELEASE);
#endif
        meta->used_blocks--;
        break;
    default:
        LOG("[WARN] block id %zu status invalid %d,who make this?", block_id, free_block.used);
        break;
    }
    LOG("[INFO] block id %zu freed", block_id);
    spin_unlock(&(meta->lock));
}

#endif
