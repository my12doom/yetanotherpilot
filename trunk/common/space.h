#include <stdio.h>

// implement these
#define page_size 2048
#define buffer_size 8196
int space_raw_write(int address, const void *data, int size);
int space_raw_read(int address, void *data, int size);
int space_raw_erase(int address);
int space_raw_init();

// use these
int space_init();
int space_read(const void *key, int keysize, void *data, int num_to_read, int *num_read);
int space_write(const void *key, int keysize, const void *data, int num_to_write, int *num_written);
int space_delete(const void *key, int keysize);
int space_resort();
int space_available();
