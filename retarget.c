#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <stddef.h>
#include <string.h>
#include "gd32vf103.h"

void write_hex(int fd, unsigned long int hex)
{
    uint8_t ii;
    uint8_t jj;
    char towrite;
    write(fd , "0x", 2);
    for (ii = sizeof(unsigned long int) * 2 ; ii > 0; ii--) {
        jj = ii - 1;
        uint8_t digit = ((hex & (0xF << (jj*4))) >> (jj*4));
        towrite = digit < 0xA ? ('0' + digit) : ('A' +  (digit - 0xA));
        write(fd, &towrite, 1);
    }
}

static inline int _stub(int err)
{
    return -1;
}

void _exit(int code)
{
    const char message[] = "\nProgram has exited with code:";

    write(STDERR_FILENO, message, sizeof(message) - 1);
    write_hex(STDERR_FILENO, code);
    write(STDERR_FILENO, "\n", 1);

    for (;;);
}

int _close(int fd)
{
    return _stub(EBADF);
}

int _fstat(int fd, struct stat* st)
{
    if (isatty(fd)) {
        st->st_mode = S_IFCHR;
        return 0;
    }

    return _stub(EBADF);
}

int _isatty(int fd)
{
    if (fd == STDOUT_FILENO || fd == STDERR_FILENO)
        return 1;

    return 0;
}

off_t _lseek(int fd, off_t ptr, int dir)
{
    if (isatty(fd))
        return 0;

    return _stub(EBADF);
}

ssize_t _read(int fd, void* ptr, size_t len)
{
    return _stub(EBADF);
}

void *_sbrk(ptrdiff_t incr)
{
    extern char _stack_end[];
    extern char _heap_start[];
    static char *curbrk = _stack_end;

    if ((curbrk + incr < _stack_end) || (curbrk + incr > _heap_start))
        return NULL - 1;

    curbrk += incr;
    return curbrk - incr;
}

typedef unsigned int size_t;

/* retarget the C library printf function to the USART */
int _put_char(int ch)
{
    usart_data_transmit(USART0, (uint8_t) ch );
    while ( usart_flag_get(USART0, USART_FLAG_TBE)== RESET) {}

    return ch;
}

ssize_t _write(int fd, const void* ptr, size_t len) {
    const uint8_t * current = (const uint8_t *) ptr;

    for (size_t jj = 0; jj < len; jj++) {
        _put_char(current[jj]);

        if (current[jj] == '\n') {
            _put_char('\r');
        }
    }

    return len;
}

int puts(const char* string) {
    return _write(0, (const void *) string, strlen(string));
}

