//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzgeqp3.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 02-Jan-2023 15:29:42
//

#ifndef XZGEQP3_H
#define XZGEQP3_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace reflapack {
void qrf(::coder::array<double, 2U> &A, int m, int n, int nfxd,
         ::coder::array<double, 1U> &tau);

}
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xzgeqp3.h
//
// [EOF]
//
