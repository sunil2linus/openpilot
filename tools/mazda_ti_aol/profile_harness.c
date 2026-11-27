// Runs one extracted aarch64 function `bool f(uint16_t)` (position independent, no calls or loads) for every
// uint16 value and prints the values it accepts, one per line. Built here as a static aarch64 program and run with
// qemu-aarch64 by check_pandad.py; never run on the device.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  FILE *f = fopen(argv[1], "rb");
  if (f == NULL) return 2;
  unsigned char code[4096];
  size_t n = fread(code, 1, sizeof(code), f);
  fclose(f);
  if (n == 0 || n >= sizeof(code)) return 2;
  void *page = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  if (page == MAP_FAILED) return 2;
  memcpy(page, code, n);
  if (mprotect(page, 4096, PROT_READ | PROT_EXEC) != 0) return 2;
  __builtin___clear_cache((char *)page, (char *)page + n);
  _Bool (*fn)(uint16_t) = (_Bool (*)(uint16_t))page;
  for (uint32_t v = 0; v <= 0xFFFFU; v++) {
    if (fn((uint16_t)v)) printf("%u\n", v);
  }
  return 0;
}
