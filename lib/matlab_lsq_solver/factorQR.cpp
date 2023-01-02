//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: factorQR.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 02-Jan-2023 15:29:42
//

// Include Files
#include "factorQR.h"
#include "solve_internal_types.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : c_struct_T *obj
//                const ::coder::array<double, 1U> &A
//                int mrows
//                int ncols
//                int ldA
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace QRManager {
void factorQR(c_struct_T *obj, const ::coder::array<double, 1U> &A, int mrows,
              int ncols, int ldA)
{
  int ix0;
  int iy0;
  int minmana;
  boolean_T guard1{false};
  iy0 = mrows * ncols;
  guard1 = false;
  if (iy0 > 0) {
    for (minmana = 0; minmana < ncols; minmana++) {
      ix0 = ldA * minmana;
      iy0 = obj->ldq * minmana;
      for (int k{0}; k < mrows; k++) {
        obj->QR[iy0 + k] = A[ix0 + k];
      }
    }
    guard1 = true;
  } else if (iy0 == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }
  if (guard1) {
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    for (minmana = 0; minmana < ncols; minmana++) {
      obj->jpvt[minmana] = minmana + 1;
    }
    if (mrows <= ncols) {
      iy0 = mrows;
    } else {
      iy0 = ncols;
    }
    obj->minRowCol = iy0;
    ix0 = obj->QR.size(0);
    minmana = obj->QR.size(1);
    if (ix0 <= minmana) {
      minmana = ix0;
    }
    obj->tau.set_size(minmana);
    for (ix0 = 0; ix0 < minmana; ix0++) {
      obj->tau[ix0] = 0.0;
    }
    if (iy0 >= 1) {
      internal::reflapack::qrf(obj->QR, mrows, ncols, iy0, obj->tau);
    }
  }
}

} // namespace QRManager
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for factorQR.cpp
//
// [EOF]
//
