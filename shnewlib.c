// semi-hosted newlib

#include <reent.h>
#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>

#define REENTRANCY

#ifdef errno
    #undef errno
    extern int errno;
#endif

/* File operations */
#define SYS_OPEN        0x01 // Open a file or stream on the host system.
#define SYS_ISTTY       0x09 // Check whether a file handle is associated with a file or a stream/terminal such as stdout.
#define SYS_WRITE       0x05 // Write to a file or stream.
#define SYS_READ        0x06 // Read from a file at the current cursor position.
#define SYS_CLOSE       0x02 // Closes a file on the host which has been opened by SYS_OPEN.
#define SYS_FLEN        0x0C // Get the length of a file.
#define SYS_SEEK        0x0A // Set the file cursor to a given position in a file.
#define SYS_TMPNAM      0x0D // Get a temporary absolute file path to create a temporary file.
#define SYS_REMOVE      0x0E // Remove a file on the host system. Possibly insecure!
#define SYS_RENAME      0x0F // Rename a file on the host system. Possibly insecure!

/* Terminal I/O operations */
#define SYS_WRITEC      0x03 // Write one character to the debug terminal.
#define SYS_WRITE0      0x04 // Write a 0-terminated string to the debug terminal.
#define SYS_READC       0x07 // Read one character from the debug terminal.

/* Time operations */
#define SYS_CLOCK       0x10
#define SYS_ELAPSED     0x30
#define SYS_TICKFREQ    0x31
#define SYS_TIME        0x11

/* System/Misc. operations */
#define SYS_ERRNO       0x13 // Returns the value of the C library errno variable that is associated with the semihosting implementation.
#define SYS_GET_CMDLINE 0x15 // Get commandline parameters for the application to run with (argc and argv for main())
#define SYS_HEAPINFO    0x16
#define SYS_ISERROR     0x08
#define SYS_SYSTEM      0x12

typedef uintptr_t semihost_argt;
typedef intptr_t semihost_rett;
typedef struct _reent reent;

#define HANDLE_STDIN 0
#define HANDLE_STDOUT 1
#define HANDLE_STDERR 2

typedef struct sys_seek_t
{
    int handle;
    const char* buff;
    int size;
} sys_seek_t;

static semihost_rett __attribute__ ((noinline)) __semihost(unsigned int swi, semihost_argt arg)
{
    asm("bkpt 0xab");
}

static void _seterrno(int* _errno)
{
    if (_errno == 0)
    {
        errno = __semihost(SYS_ERRNO, 0);
    }
    else
    {
        *_errno = __semihost(SYS_ERRNO, 0);
    }
}

static semihost_rett semihost(unsigned int swi, semihost_argt arg, int* _errno)
{
    int retv = __semihost(swi, arg);
    switch(swi)
    {
    case SYS_READ:
        {
            sys_seek_t* block = (sys_seek_t*)(void*)(arg);
            if (retv < 0)
            {
                _seterrno(_errno);
            }
            else if (block == 0)
            {
                // probably this is the right thing to do?
                _seterrno(_errno);
            }
            else if (retv >= block->size)
            { 
                _seterrno(_errno);
            }
        }
        break;
    case SYS_OPEN:
    case SYS_SEEK:
        if (retv < 0) _seterrno(_errno);
        break;
    case SYS_REMOVE:
    case SYS_CLOSE:
    case SYS_WRITE:
        if (retv != 0) _seterrno(_errno);
        break;
    }
    
    return retv;
}

static void __attribute__((noreturn)) __exit_with_msg(const char* msg)
{
    __semihost(SYS_WRITE0, (semihost_argt)(void*)msg);
    volatile float a = 0.0f;
    asm(".byte 0xDE, 0xDE"); // permanently undefined instruction.
    __semihost(1000, 0); // should be a fault
    while (1) asm("nop");
}

#ifdef REENTRANCY
    #define REENTRANT(n) n##_r
#else
    #define REENTRANT(n) n##_r_
#endif

#define REENT_WRAPPER(rt, name, ...)\
    reent r; \
    r._errno = 0xBEEF; \
    rt rv = REENTRANT(name)(&r, ##__VA_ARGS__); \
    if (r._errno != 0xBEEF) errno = r._errno; \
    return rv;

void __attribute__((noreturn)) _exit(int code)
{
    char buff[100];
    memset(buff, 0, sizeof(buff));
    snprintf(buff, 99, "exit status %d", code);
    __exit_with_msg(buff);
}

int REENTRANT(_close)(reent* r, int file)
{
    return -1;
}

int _close(int file)
{
    REENT_WRAPPER(int, _close, file);
}

static char *__env[1] = { 0 };
char **_environ = __env;

int REENTRANT(_execve)(reent* r, const char* name, char* const * argv, char* const * env)
{
    r->_errno = ENOMEM;
    return -1;
}

int _execve(const char* name, char* const * argv, char* const * env)
{
    REENT_WRAPPER(int, _execve, name, argv, env);
}

int REENTRANT(_fork)(reent* r)
{
    r->_errno = EAGAIN;
    return -1;
}

int _fork(void)
{
    REENT_WRAPPER(int, _fork);
}

int REENTRANT(_fstat)(reent* r, int file, struct stat* st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _fstat(int file, struct stat* st)
{
    REENT_WRAPPER(int, _fstat, file, st);
}

int REENTRANT(_getpid)(reent* r)
{
    return 1;
}

int _getpid(void)
{
    REENT_WRAPPER(int, _getpid);
}

int REENTRANT(_isatty)(reent* r, int file)
{
    return 1;
}

int _isatty(int file)
{
    REENT_WRAPPER(int, _isatty, file);
}

int REENTRANT(_kill)(reent* r, int pid, int sig)
{
    r->_errno = EINVAL;
    return -1;
}

int _kill(int pid, int sig)
{
    REENT_WRAPPER(int, _kill, pid, sig);
}

int REENTRANT(_link)(reent* r, const char* old, const char* new)
{
    r->_errno = EMLINK;
    return -1;
}

int _link(const char* old, const char* new)
{
    REENT_WRAPPER(int, _link, old, new);
}

off_t REENTRANT(_lseek)(reent* r, int file, off_t ptr, int dir)
{
    return 0;
}

off_t _lseek(int file, off_t ptr, int dir)
{
    REENT_WRAPPER(off_t, _lseek, file, ptr, dir);
}

int REENTRANT(_open)(reent* r, const char* name, int flags, int mode)
{
    return -1;
}

int _open(const char* name, int flags, int mode)
{
    REENT_WRAPPER(int, _open, name, flags, mode);
}

ssize_t REENTRANT(_read)(reent* r, int file, void* ptr, size_t len)
{
    return 0;
}

ssize_t _read(int file, void* ptr, size_t len)
{
    REENT_WRAPPER(ssize_t, _read, file, ptr, len);
}

extern char __end__;
static char* heap_end = 0;

void* REENTRANT(_sbrk)(reent* r, ptrdiff_t incr)
{
    register char* stack asm("sp");
    char* prev_heap_end;
    if (heap_end == 0)
    {
        heap_end = &__end__;
    }
    prev_heap_end = heap_end;
    if (heap_end + incr > stack)
    {
        __exit_with_msg("Heap and stack collision.\n");
    }
    
    heap_end += incr;
    return (void*)prev_heap_end;
}

void* _sbrk(ptrdiff_t incr)
{
    REENT_WRAPPER(void*, _sbrk, incr);
}

int REENTRANT(_stat)(reent* r, const char* file, struct stat* st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _stat(const char* file, struct stat* st)
{
    REENT_WRAPPER(int, _stat, file, st);
}

clock_t REENTRANT(_times)(reent* r, struct tms* buf)
{
    return -1;
}

clock_t _times(struct tms* buf)
{
    REENT_WRAPPER(clock_t, _times, buf);
}

int REENTRANT(_unlink)(reent* r, const char* name)
{
    r->_errno = ENOENT;
    return -1;
}

int _unlink(const char* name)
{
    REENT_WRAPPER(int, _unlink, name);
}

int REENTRANT(_wait)(reent* r, int* status)
{
    r->_errno = ECHILD;
    return -1;
}

int _wait(int* status)
{
    REENT_WRAPPER(int, _wait, status);
}

ssize_t REENTRANT(_write)(reent* r, int file, const void* ptr, size_t len)
{
    if (ptr == 0 || len <= 0 || file < 0)
    {
        r->_errno = EINVAL;
        return -1;
    }
    if (file == HANDLE_STDOUT || file == HANDLE_STDERR)
	{
        char buff[len+1];
        memcpy(buff, ptr, len);
        buff[len] = 0;
        semihost(SYS_WRITE0, (semihost_argt)(void*)buff, &r->_errno);
    }
    else
    {
        sys_seek_t seek =
        {
            .handle = file,
            .buff = ptr,
            .size = len
        };
        semihost(SYS_WRITE, (semihost_argt)(void*)&seek, &r->_errno);
    }
}

ssize_t _write(int file, const void* ptr, size_t len)
{
    REENT_WRAPPER(ssize_t, _write, file, ptr, len);
}