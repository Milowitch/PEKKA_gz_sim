#ifndef LKM_DRIVER_COMMON_HH
#define LKM_DRIVER_COMMON_HH

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

#include <vector>
#include <cstdint>

namespace lkm_m5 {

typedef std::vector<uint8_t> Frame;

}

#endif // !LKM_DRIVER_COMMON_HH
