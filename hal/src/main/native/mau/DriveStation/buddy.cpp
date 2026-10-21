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
    head.status = free;
    head.k = m;
    InsertAfter(sentinel[m], head);
}

BuddyPool::~BuddyPool(){
    delete [] pool;
}

void *BuddyPool::Acquire(size_t bytes){
    unsigned int kPrime = Log2Ceil(bytes + sizeof(Header));

    unsigned int i = kPrime;
    while (i <= m && sentinel[i].link.next == &sentinel[i]) {
        ++i;
    }
    if (i > m) {
        return NULL;    // throw bad_alloc("out of memory");
    }

    Block &block = *sentinel[i].link.next;
    Unlink(block);
    while (block.k > kPrime) {
        block.k -= 1;
        Block &buddy = Buddy(block);
        buddy.status = free;
        buddy.k = block.k;
        InsertAfter(sentinel[buddy.k], buddy);
    }
    block.status = reserved;
    return block.userPart;
}

void BuddyPool::Release(void *arg){
    Block &block = *reinterpret_cast<Block *>(
                                              reinterpret_cast<Header *>(arg) - 1U);

    if (&block < pool || &block >= pool + numberOfBlocks) {
        return; // throw invalid_argument("invalid pointer");
    }

    if (block.status != reserved) {
    	return;
    }

    block.status = free;
    Block *ptr;
    for (ptr = &block; ptr->k < m; ptr->k += 1) {
        Block &buddy = Buddy(*ptr);
        if (buddy.status == reserved || buddy.k != ptr->k) {
            break;
        }
        Unlink(buddy);
        if (&buddy < ptr) {
            ptr = &buddy;
        }
    }

    InsertAfter(sentinel[ptr->k], *ptr);
}

BuddyPool::Block &BuddyPool::Buddy(Block &block) const{
    uintptr_t addr = reinterpret_cast<uintptr_t>(&block) + (1 << block.k);
    return *(reinterpret_cast<Block *>(addr));
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
