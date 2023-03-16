//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: trisolve.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 14-Mar-2023 23:05:22
//

#ifndef TRISOLVE_H
#define TRISOLVE_H

// Include Files
#include "../rtwtypes.h"
#include "../coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void b_trisolve(const ::coder::array<double, 2U> &A,
                ::coder::array<double, 1U> &B);

void trisolve(const ::coder::array<double, 2U> &A,
              ::coder::array<double, 1U> &B);

} // namespace internal
} // namespace coder

#endif
//
// File trailer for trisolve.h
//
// [EOF]
//
