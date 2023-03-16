//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: qpipmw.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 14-Mar-2023 23:05:22
//

#ifndef QPIPMW_H
#define QPIPMW_H

// Include Files
#include "../rtwtypes.h"
#include "../coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void b_plus(coder::array<double, 1U> &in1, const coder::array<double, 1U> &in2);

void binary_expand_op(coder::array<double, 1U> &in1,
                      const coder::array<double, 2U> &in2,
                      const coder::array<double, 1U> &in3);

void binary_expand_op(coder::array<double, 1U> &in1,
                      const coder::array<double, 1U> &in2, const int in3[2]);

void binary_expand_op(coder::array<double, 1U> &in1,
                      const coder::array<double, 1U> &in2, double in3,
                      const coder::array<double, 1U> &in4,
                      const coder::array<double, 1U> &in5, double in6,
                      const coder::array<double, 1U> &in7);

namespace coder {
void kkt_fact(const ::coder::array<double, 2U> &Q,
              const ::coder::array<double, 2U> &A,
              const ::coder::array<double, 2U> &E,
              const ::coder::array<double, 1U> &z,
              const ::coder::array<double, 1U> &s,
              ::coder::array<double, 2U> &L, ::coder::array<double, 2U> &U,
              ::coder::array<double, 2U> &pp);

void kkt_residual(
    const ::coder::array<double, 2U> &Q, const ::coder::array<double, 1U> &c,
    const ::coder::array<double, 2U> &A, const ::coder::array<double, 1U> &b,
    const ::coder::array<double, 2U> &E, const ::coder::array<double, 1U> &x,
    const ::coder::array<double, 1U> &z, const ::coder::array<double, 1U> &s,
    ::coder::array<double, 1U> &rQ, ::coder::array<double, 1U> &rA,
    ::coder::array<double, 1U> &rS);

void kkt_solve(
    const ::coder::array<double, 2U> &A, const ::coder::array<double, 1U> &rQ,
    const ::coder::array<double, 1U> &rA, const ::coder::array<double, 1U> &rS,
    const ::coder::array<double, 1U> &z, const ::coder::array<double, 1U> &s,
    const ::coder::array<double, 2U> &L, const ::coder::array<double, 2U> &U,
    const ::coder::array<double, 2U> &pp, ::coder::array<double, 1U> &dx,
    ::coder::array<double, 1U> &dz, ::coder::array<double, 1U> &ds);

} // namespace coder
void plus(coder::array<double, 1U> &in1, const coder::array<double, 1U> &in2);

#endif
//
// File trailer for qpipmw.h
//
// [EOF]
//
