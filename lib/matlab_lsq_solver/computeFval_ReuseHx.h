//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeFval_ReuseHx.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 02-Jan-2023 15:29:42
//

#ifndef COMPUTEFVAL_REUSEHX_H
#define COMPUTEFVAL_REUSEHX_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct b_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
double computeFval_ReuseHx(const b_struct_T *obj,
                           ::coder::array<double, 2U> &workspace,
                           const ::coder::array<double, 1U> &f,
                           const ::coder::array<double, 1U> &x);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for computeFval_ReuseHx.h
//
// [EOF]
//
