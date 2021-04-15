#include "gtest/gtest.h"
#include "opencv2/core.hpp"

namespace opencv_assert {

static inline ::testing::AssertionResult CvMatCompare(const char* a_expr,
                                                      const char* b_expr,
                                                      const cv::Mat &a,
                                                      const cv::Mat &b,
                                                      const bool negate)
{
    if (a.rows != b.rows) {
        if (negate) {
            return ::testing::AssertionSuccess();
        }

        return ::testing::AssertionFailure() << a_expr << " and " << b_expr
                                             << " have a different number of rows (a: "
                                             << a.rows << ", b: " << b.rows << ")";
    }

    if (a.dims != b.dims) {
        if (negate) {
            return ::testing::AssertionSuccess();
        }

        return ::testing::AssertionFailure() << a_expr << " and " << b_expr
                                             << " have different dimensions (a: "
                                             << a.dims << ", b: " << b.dims
                                             << ").";
    }

    if (a.size != b.size) {
        if (negate) {
            return ::testing::AssertionSuccess();
        }

        return ::testing::AssertionFailure() << a_expr << " and " << b_expr
                                             << " have different sizes (a: "
                                             << a.size << ", b: " << b.size
                                             << ").";
    }

    if (a.elemSize() != b.elemSize()) {
        if (negate) {
            return ::testing::AssertionSuccess();
        }

        return ::testing::AssertionFailure() << a_expr << " and " << b_expr
                                             << " have different element sizes (a:"
                                             << a.elemSize() << ", b: " << b.elemSize()
                                             << ").";
    }

    if (a.isContinuous() && b.isContinuous()) {
        if (0 != memcmp(a.ptr(), b.ptr(), a.total() * a.elemSize())) {
            if (negate) {
                return ::testing::AssertionSuccess();
            }

            return ::testing::AssertionFailure() << a_expr << " and " << b_expr
                                                 << " differ";
        }

        if (negate) {
            return ::testing::AssertionFailure() << a_expr << " and " << b_expr
                                                << " don't differ.";
        }

        return ::testing::AssertionSuccess();
    }

    const cv::Mat *arrays[] = {&a, &b, 0};
    uchar *ptrs[2];
    cv::NAryMatIterator it(arrays, ptrs, 2);
    for (unsigned int p = 0; p < it.nplanes; p++, ++it) {
        if (0 != memcmp(it.ptrs[0], it.ptrs[1], it.size*a.elemSize())) {
            if (negate) {
                return ::testing::AssertionSuccess();
            }

            return ::testing::AssertionFailure() << a_expr << " and " << b_expr
                                                 << " differ.";
        }
    }

    if (negate) {
        return ::testing::AssertionFailure() << a_expr << " and " << b_expr
                                                << " don't differ.";
    }

    return ::testing::AssertionSuccess();
}

static inline ::testing::AssertionResult CvMatEq(const char* a_expr,
                                          const char* b_expr,
                                          const cv::Mat &a,
                                          const cv::Mat &b)
{
    return CvMatCompare(a_expr, b_expr, a, b, false);
}

static inline ::testing::AssertionResult CvMatNe(const char* a_expr,
                                          const char* b_expr,
                                          const cv::Mat &a,
                                          const cv::Mat &b)
{
    return CvMatCompare(a_expr, b_expr, a, b, true);
}

} // opencv_assert
