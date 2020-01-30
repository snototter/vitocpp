#ifndef __VCP_UTILS_TEST_UTILS_H__
#define __VCP_UTILS_TEST_UTILS_H__

#include <gtest/gtest.h>
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/core/core.hpp>
#else
    #include <opencv2/core.hpp>
#endif

namespace vcp {
namespace utils {
namespace test {
} // namespace test
} // namespace utils
} // namespace vcp

#endif // __VCP_UTILS_TEST_UTILS_H__
