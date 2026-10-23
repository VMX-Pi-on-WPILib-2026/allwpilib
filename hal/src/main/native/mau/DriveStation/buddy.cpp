#include <cstdint>
//#include <WinDef.h>
#include "buddy.h"

unsigned int Log2Ceil(unsigned int val){
    unsigned int L;
    for (L = 0; (1ul<<L) < val; L++);
    return L;
}

BuddyPool::BuddyPool(size_t bytes)
    : m(Log2Ceil(bytes))
    , numberOfBlocks( (1 << m) / sizeof(Block))
    , pool (new Block[numberOfBlocks + m +1])
    , sentinel(pool + numberOfBlocks)
{
    for (unsigned int i = 0; i <= m; ++i) {
        sentinel[i].link.next = &sentinel[i];
        sentinel[i].link.prev = &sentinel[i];
    }

    Block &head = pool[0];
    head.header.status = free;
    head.header.k = m;
    InsertAfter(sentinel[m], head);
}

BuddyPool::~BuddyPool(){
    delete [] pool;
}

void *BuddyPool::Acquire(size_t bytes){
    unsigned int kPrime = Log2Ceil(bytes + sizeof(Header));
    // Every allocation is represented by a full Block object in the pool.
    // Never split below that size, even when the requested payload is small.
    const unsigned int minBlockLog2 = Log2Ceil(sizeof(Block));
    if (kPrime < minBlockLog2) {
        kPrime = minBlockLog2;
    }

    unsigned int i = kPrime;
    while (i <= m && sentinel[i].link.next == &sentinel[i]) {
        ++i;
    }
    if (i > m) {
        return NULL;    // throw bad_alloc("out of memory");
    }

    Block &block = *sentinel[i].link.next;
    Unlink(block);
    while (block.header.k > kPrime) {
        block.header.k -= 1;
        Block &buddy = Buddy(block);
        buddy.header.status = free;
        buddy.header.k = block.header.k;
        InsertAfter(sentinel[buddy.header.k], buddy);
    }
    block.header.status = reserved;
    return block.userPart;
}

void BuddyPool::Release(void *arg){
    Block &block = *reinterpret_cast<Block *>(
                                              reinterpret_cast<Header *>(arg) - 1U);

    if (&block < pool || &block >= pool + numberOfBlocks) {
        return; // throw invalid_argument("invalid pointer");
    }

    if (block.header.status != reserved) {
    	return;
    }

    block.header.status = free;
    Block *ptr;
    for (ptr = &block; ptr->header.k < m; ptr->header.k += 1) {
        Block &buddy = Buddy(*ptr);
        if (buddy.header.status == reserved || buddy.header.k != ptr->header.k) {
            break;
        }
        Unlink(buddy);
        if (&buddy < ptr) {
            ptr = &buddy;
        }
    }

    InsertAfter(sentinel[ptr->header.k], *ptr);
}

BuddyPool::Block &BuddyPool::Buddy(Block &block) const{
    // Buddy blocks are adjacent within each 2^k-byte region. Adding the block
    // size only finds the buddy for a left-hand block; for a right-hand block
    // it walks into the next region (and can leave the pool entirely). Toggle
    // the k-th address bit relative to the pool base to find either side.
    uintptr_t base = reinterpret_cast<uintptr_t>(pool);
    uintptr_t offset = reinterpret_cast<uintptr_t>(&block) - base;
    uintptr_t buddyOffset = offset ^ (uintptr_t{1} << block.header.k);
    return *reinterpret_cast<Block *>(base + buddyOffset);
}

void BuddyPool::Unlink(Block &block){
    if (block.link.next) {
        block.link.next->link.prev = block.link.prev;
    }
    if (block.link.prev) {
        block.link.prev->link.next = block.link.next;
    }
    block.link.next = block.link.prev = &block;
}

void BuddyPool::InsertAfter(Block &src, Block &block){
    block.link.next = src.link.next;
    block.link.prev = &src;

    if (src.link.next) {
        src.link.next->link.prev = &block;
    }
    src.link.next = &block;
}
