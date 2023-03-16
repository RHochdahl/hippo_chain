//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: trisolve.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 14-Mar-2023 23:05:22
//

// Include Files
#include "trisolve.h"
#include "../rt_nonfinite.h"
#include "../coder_array.h"

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &A
//                ::coder::array<double, 1U> &B
// Return Type  : void
//
namespace coder {
namespace internal {
void b_trisolve(const ::coder::array<double, 2U> &A,
                ::coder::array<double, 1U> &B)
{
  int b_u1;
  int kAcol;
  int u1;
  kAcol = A.size(0);
  u1 = A.size(1);
  if (kAcol <= u1) {
    u1 = kAcol;
  }
  b_u1 = B.size(0);
  if (u1 <= b_u1) {
    b_u1 = u1;
  }
  if (B.size(0) != 0) {
    for (u1 = b_u1; u1 >= 1; u1--) {
      double d;
      kAcol = A.size(0) * (u1 - 1);
      d = B[u1 - 1];
      if (d != 0.0) {
        B[u1 - 1] = d / A[(u1 + kAcol) - 1];
        for (int i{0}; i <= u1 - 2; i++) {
          B[i] = B[i] - B[u1 - 1] * A[i + kAcol];
        }
      }
    }
  }
}

//
// Arguments    : const ::coder::array<double, 2U> &A
//                ::coder::array<double, 1U> &B
// Return Type  : void
//
void trisolve(const ::coder::array<double, 2U> &A,
              ::coder::array<double, 1U> &B)
{
  int b_u1;
  int kAcol;
  int u1;
  kAcol = A.size(0);
  u1 = A.size(1);
  if (kAcol <= u1) {
    u1 = kAcol;
  }
  b_u1 = B.size(0);
  if (u1 <= b_u1) {
    b_u1 = u1;
  }
  if (B.size(0) != 0) {
    for (int k{0}; k < b_u1; k++) {
      kAcol = A.size(0) * k;
      if (B[k] != 0.0) {
        B[k] = B[k] / A[k + kAcol];
        u1 = k + 2;
        for (int i{u1}; i <= b_u1; i++) {
          B[i - 1] = B[i - 1] - B[k] * A[(i + kAcol) - 1];
        }
      }
    }
  }
}

} // namespace internal
} // namespace coder

//
// File trailer for trisolve.cpp
//
// [EOF]
//
