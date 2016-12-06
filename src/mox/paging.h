

typedef struct MemPage_s {
    struct MemPage_s *previous, *next;
    unsigned char virtualAddr;
    unsigned int flags;
} MemPage;


MemPage mox_mempage_alloc(unsigned char baseAddr);
MemPage mox_mempage_free(unsigned char baseAddr);
MemPage mox_mempage_read(unsigned char baseAddr);
