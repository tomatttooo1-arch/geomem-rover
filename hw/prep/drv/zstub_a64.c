/* DRV5 test-only: the host has no aarch64 zlib; dpu_rx.cpp (linked for letterbox()) references crc32, which the
 * driver test never calls. Linked into the qemu test binary only, never into dpu_rx. */
#include <stdlib.h>
unsigned long crc32(unsigned long crc, const unsigned char* buf, unsigned int len) { (void)crc; (void)buf; (void)len; abort(); }
