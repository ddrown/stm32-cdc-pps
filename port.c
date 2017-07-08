#include <stdio.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main() {
  int fd;
  struct serial_icounter_struct counters;

  fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
  printf("fd = %d\n", fd);
  int ret = ioctl(fd, TIOCGICOUNT, &counters);
  printf("ret = %d\n", ret);

  printf("cts = %d\n", counters.cts);
  printf("dsr = %d\n", counters.dsr);
  printf("rng = %d\n", counters.rng);
  printf("dcd = %d\n", counters.dcd);
  printf("rx  = %d\n", counters.rx);
  printf("tx  = %d\n", counters.tx);
  printf("fram= %d\n", counters.frame);
  printf("over= %d\n", counters.overrun);
  printf("pari= %d\n", counters.parity);
  printf("brk = %d\n", counters.brk);
  printf("buff= %d\n", counters.buf_overrun);
}
