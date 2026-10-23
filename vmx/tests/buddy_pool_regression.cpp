#include "buddy.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

struct Allocation {
    void* data;
    std::size_t size;
    std::uint8_t value;
};

int main() {
    BuddyPool pool(1U << 16);
    std::mt19937 random(0x564d58);
    std::vector<Allocation> active;

    for (int i = 0; i < 100000; ++i) {
        const bool allocate = active.empty() || (random() % 3 != 0);
        if (allocate) {
            const std::size_t size = 1 + random() % 4096;
            void* data = pool.Acquire(size);
            if (data) {
                const auto value = static_cast<std::uint8_t>(random());
                auto* bytes = static_cast<std::uint8_t*>(data);
                for (std::size_t j = 0; j < size; ++j) {
                    bytes[j] = value;
                }
                active.push_back({data, size, value});
            }
        } else {
            const std::size_t index = random() % active.size();
            const Allocation allocation = active[index];
            auto* bytes = static_cast<const std::uint8_t*>(allocation.data);
            for (std::size_t j = 0; j < allocation.size; ++j) {
                assert(bytes[j] == allocation.value);
            }
            pool.Release(allocation.data);
            active[index] = active.back();
            active.pop_back();
        }
    }

    for (const Allocation& allocation : active) {
        auto* bytes = static_cast<const std::uint8_t*>(allocation.data);
        for (std::size_t j = 0; j < allocation.size; ++j) {
            assert(bytes[j] == allocation.value);
        }
        pool.Release(allocation.data);
    }
}
