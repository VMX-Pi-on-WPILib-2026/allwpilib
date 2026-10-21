#ifndef NATIVE_MAUTIME_H

#include <VMXTime.h>

#define NATIVE_MAUTIME_H

namespace mau {
    extern VMXTime* vmxTime;

    inline uint64_t vmxGetTime() {
        return vmxTime->GetCurrentTotalMicroseconds();
    }
}

#endif //NATIVE_MAUTIME_H
