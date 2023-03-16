//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: maxConstraintViolation.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "maxConstraintViolation.h"
#include "../rt_nonfinite.h"
#include "solveLSQ_internal_types.h"
#include "../coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const f_struct_T *obj
//                const ::coder::array<double, 1U> &x
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
double maxConstraintViolation(const f_struct_T *obj,
                              const ::coder::array<double, 1U> &x)
{
  double v;
  int mFixed;
  int mLB;
  int mUB;
  mLB = obj->sizes[3];
  mUB = obj->sizes[4];
  mFixed = obj->sizes[0];
  v = 0.0;
  if (obj->sizes[3] > 0) {
    for (int idx{0}; idx < mLB; idx++) {
      int idxLB;
      idxLB = obj->indexLB[idx] - 1;
      v = std::fmax(v, -x[idxLB] - obj->lb[idxLB]);
    }
  }
  if (obj->sizes[4] > 0) {
    for (int idx{0}; idx < mUB; idx++) {
      v = std::fmax(v, x[obj->indexUB[idx] - 1] - 1.0);
    }
  }
  if (obj->sizes[0] > 0) {
    for (int idx{0}; idx < mFixed; idx++) {
      v = std::fmax(v, std::abs(x[obj->indexFixed[idx] - 1] - 1.0));
    }
  }
  return v;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for maxConstraintViolation.cpp
//
// [EOF]
//
