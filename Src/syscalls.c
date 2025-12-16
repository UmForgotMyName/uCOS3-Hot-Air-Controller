// Src/syscalls.c
#include <sys/stat.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>

// These symbols come from your linker script (STM32F429ZI_FLASH.ld).
// _end marks the end of .bss/.data (start of heap).
// _estack is the top of RAM (start of stack, grows downward).
extern uint8_t _end;     // Provided by linker script
extern uint8_t _estack;  // Provided by linker script

static uint8_t *heap_end;

caddr_t _sbrk(int incr) {
    if (heap_end == 0) {
        heap_end = &_end;
    }
    uint8_t *prev = heap_end;
    uint8_t *next = heap_end + incr;

    // Very conservative guard: do not cross into stack.
    // Leave 4 KB safety margin to avoid accidental collision.
    if (next >= ((uint8_t *)&_estack - 4096)) {
        errno = ENOMEM;
        return (caddr_t)-1;
    }
    heap_end = next;
    return (caddr_t)prev;
}

// Minimal stubs to satisfy newlib-nano reentrant wrappers.
// If you later want printf via UART/USB, replace _write with your driver.

int _close(int fd) {
    (void)fd;
    errno = ENOSYS;
    return -1;
}

int _fstat(int fd, struct stat *st) {
    (void)fd;
    if (st) {
        st->st_mode = S_IFCHR; // Character device
    }
    return 0;
}

int _isatty(int fd) {
    (void)fd;
    return 1;
}

int _lseek(int fd, int ptr, int dir) {
    (void)fd; (void)ptr; (void)dir;
    errno = ENOSYS;
    return -1;
}

int _read(int fd, char *buf, int len) {
    (void)fd; (void)buf; (void)len;
    errno = ENOSYS;
    return -1;
}

int _kill(int pid, int sig) {
    (void)pid; (void)sig;
    errno = ENOSYS;
    return -1;
}

int _getpid(void) {
    return 1;
}

void _exit(int status) {
    (void)status;
    while (1) { /* spin */ }
}

int _open(const char *name, int flags, int mode) {
    (void)name; (void)flags; (void)mode;
    errno = ENOSYS;
    return -1;
}

int _rename(const char *oldpath, const char *newpath) {
    (void)oldpath; (void)newpath;
    errno = ENOSYS;
    return -1;
}

int _unlink(const char *name) {
    (void)name;
    errno = ENOSYS;
    return -1;
}
