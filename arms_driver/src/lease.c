#include "arms/lease.h"
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
int arms_acquire_lease(void)
{
    int fd = open("/run/lock/bxi-ethercat-arms.lock", O_NOFOLLOW | O_CLOEXEC | O_RDWR | O_CREAT, 0600);
    if (fd >= 0 && flock(fd, LOCK_EX | LOCK_NB)) {
        close(fd);
        return -1;
    }
    return fd;
}
