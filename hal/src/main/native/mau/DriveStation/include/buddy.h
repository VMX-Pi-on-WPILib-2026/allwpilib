#ifndef  BUDDYPOOL_INC
#define  BUDDYPOOL_INC

#include <stddef.h>

class BuddyPool{
public:
    enum Status { free, reserved };
    struct Header
    {
        Status status: 1;
        //unsigned int k : bitsizeof(unsigned int) - 1U;
        unsigned int k : 31;
    };
    struct Block : public Header
    {
        //enum { size = 16 };

        enum { size = 64 };
        struct Links
        {
            Block *next;
            Block *prev;
        };
        union
        {
            Links link;
            char userPart [size - sizeof(Header)];
        };
    };

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
