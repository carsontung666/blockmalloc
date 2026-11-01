#define _GNU_SOURCE
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>
#include "blockmalloc/blockmalloc.h"

#define POOL_SIZE (1024 * 1024 * 2)
#define BLOCK_SIZE 64

int main(void) {
    blocks_meta_t *meta = mmap(NULL, sizeof(blocks_meta_t),
        PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    uint8_t *pool = mmap(NULL, POOL_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if (meta == MAP_FAILED || pool == MAP_FAILED) {
        perror("mmap");
        return 1;
    }
    memset(pool, 0, POOL_SIZE);
    blocks_init(meta, POOL_SIZE, BLOCK_SIZE);

    /* Case 1: zombie holder (killed by SIGKILL before waitpid). */
    int pfd[2];
    if (pipe(pfd) != 0) {
        perror("pipe");
        return 1;
    }
    pid_t pid = fork();
    if (pid == 0) {
        close(pfd[0]);
        __atomic_store_n(&meta->lock, (int64_t)getpid(), __ATOMIC_RELEASE);
        if (write(pfd[1], "x", 1) != 1)
            _exit(2);
        for (;;)
            pause();
    }
    close(pfd[1]);
    char ch = 0;
    if (read(pfd[0], &ch, 1) != 1)
        return 1;
    close(pfd[0]);
    kill(pid, SIGKILL);
    /* Wait briefly for child to become zombie WITHOUT waitpid yet, testing 'Z' detection. */
    usleep(20000);
    int64_t b0 = blocks_alloc(meta, pool);
    if (b0 < 0) {
        fprintf(stderr, "alloc failed during zombie takeover\n");
        return 1;
    }
    waitpid(pid, NULL, 0);

    /* Case 2: reaped dead holder (ESRCH) and repeated random SIGKILL under load. */
    for (int round = 0; round < 20; round++) {
        pid_t w = fork();
        if (w == 0) {
            for (;;) {
                int64_t id = blocks_alloc(meta, pool);
                if (id >= 0)
                    blocks_free(meta, pool, (uint64_t)id);
            }
        }
        usleep(2000);
        kill(w, SIGKILL);
        waitpid(w, NULL, 0);
        int64_t ids[32];
        for (int i = 0; i < 32; i++) {
            ids[i] = blocks_alloc(meta, pool);
            if (ids[i] < 0) {
                fprintf(stderr, "alloc failed after kill round %d\n", round);
                return 1;
            }
        }
        for (int i = 0; i < 32; i++)
            blocks_free(meta, pool, (uint64_t)ids[i]);
    }
    blocks_free(meta, pool, (uint64_t)b0);
    /* Reclaim blocks allocated by children killed between blocks_alloc and blocks_free,
     * and verify used_blocks matches actual per-block used bits with zero drift. */
    uint64_t actual_used = 0;
    uint64_t stride = meta->sizeof_block_head + meta->block_size;
    for (uint64_t id = 0; id < meta->malloc_blocks; id++) {
        uint8_t *bp = pool + id * stride;
        uint8_t u = 0;
        switch (meta->sizeof_block_head) {
        case 2: u = (*(uint16_t *)bp) & 1u; break;
        case 4: u = (*(uint32_t *)bp) & 1u; break;
        case 8: u = (*(uint64_t *)bp) & 1u; break;
        }
        actual_used += u;
    }
    if (meta->used_blocks != actual_used) {
        fprintf(stderr, "used_blocks drift after churn: meta=%llu actual=%llu\n",
                (unsigned long long)meta->used_blocks, (unsigned long long)actual_used);
        return 1;
    }
    for (uint64_t id = 0; id < meta->malloc_blocks; id++)
        blocks_free(meta, pool, id);
    if (meta->used_blocks != 0 || meta->free_next_id < 0) {
        fprintf(stderr, "unexpected pool state after cleanup: used=%llu free_head=%lld\n",
                (unsigned long long)meta->used_blocks, (long long)meta->free_next_id);
        return 1;
    }

    /* Case 3: corrupted free_next_id and used_blocks repaired on dead-holder takeover. */
    pid_t c3 = fork();
    if (c3 == 0) {
        __atomic_store_n(&meta->lock, (int64_t)getpid(), __ATOMIC_RELEASE);
        meta->used_blocks = 999999;
        meta->free_next_id = -1;
        _exit(0);
    }
    waitpid(c3, NULL, 0);
    int64_t r0 = blocks_alloc(meta, pool);
    if (r0 < 0 || meta->used_blocks != 1) {
        fprintf(stderr, "pool repair failed: r0=%lld used=%llu\n",
                (long long)r0, (unsigned long long)meta->used_blocks);
        return 1;
    }
    blocks_free(meta, pool, (uint64_t)r0);
    if (meta->used_blocks != 0) {
        fprintf(stderr, "expected used_blocks==0 after free, got %llu\n",
                (unsigned long long)meta->used_blocks);
        return 1;
    }

    munmap(meta, sizeof(blocks_meta_t));
    munmap(pool, POOL_SIZE);
    printf("crash_recovery: ok\n");
    return 0;
}
