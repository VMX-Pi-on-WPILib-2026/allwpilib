#ifndef  BUDDYPOOL_INC
#define  BUDDYPOOL_INC

#include <stddef.h>
#include <stdint.h>

class BuddyPool{
public:
    enum Status { free, reserved };
    struct Header
    {
        Status status: 1;
        unsigned int k : 31;
        // Pad the header to the union's alignment. Acquire() returns userPart,
        // and Release() recovers the block by subtracting exactly one Header.
        uint64_t alignmentPadding;
    };
    struct Block
    {
        enum { size = 64 };
        struct Links
        {
            Block *next;
            Block *prev;
        };
        Header header;
        union
        {
            Links link;
            char userPart [size - sizeof(Header)];
        };
    };
    static_assert(sizeof(Block) == Block::size,
                  "BuddyPool blocks must match the allocator's byte sizes");

private:
    unsigned int m;
    unsigned int numberOfBlocks;
    Block *pool;
    Block *sentinel;

    static void Unlink(Block &);
    static void InsertAfter(Block &, Block &);
    Block &Buddy(Block &) const;

public:
    BuddyPool(size_t);
    ~BuddyPool();

    void *Acquire(size_t);
    void Release(void *);
};

#endif   /* ----- #ifndef BUDDYPOOL_INC  ----- */
