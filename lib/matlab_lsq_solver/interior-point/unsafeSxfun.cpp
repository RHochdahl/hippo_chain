//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: unsafeSxfun.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 14-Mar-2023 23:05:22
//

// Include Files
#include "unsafeSxfun.h"
#include "../rt_nonfinite.h"
#include "../coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : coder::array<double, 1U> &in1
//                const coder::array<double, 2U> &in3
//                const coder::array<double, 1U> &in4
// Return Type  : void
//
void b_binary_expand_op(coder::array<double, 1U> &in1,
                        const coder::array<double, 2U> &in3,
                        const coder::array<double, 1U> &in4)
{
  coder::array<double, 1U> r;
  int i;
  int in3_idx_0;
  int stride_1_0;
  in3_idx_0 = in3.size(0);
  if (in4.size(0) == 1) {
    i = in3_idx_0;
  } else {
    i = in4.size(0);
  }
  r.set_size(i);
  stride_1_0 = (in4.size(0) != 1);
  if (in4.size(0) != 1) {
    in3_idx_0 = in4.size(0);
  }
  for (i = 0; i < in3_idx_0; i++) {
    r[i] = -1.0 - in4[i * stride_1_0];
  }
  in1.set_size(r.size(0));
  in3_idx_0 = r.size(0);
  for (i = 0; i < in3_idx_0; i++) {
    double varargin_2;
    varargin_2 = r[i];
    in1[i] = std::fmax(0.0, varargin_2);
  }
}

//
// Arguments    : coder::array<double, 1U> &in1
//                const coder::array<double, 1U> &in3
//                double in4
// Return Type  : void
//
void binary_expand_op(coder::array<double, 1U> &in1,
                      const coder::array<double, 1U> &in3, double in4)
{
  coder::array<double, 1U> b_in1;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  if (in3.size(0) == 1) {
    i = in1.size(0);
  } else {
    i = in3.size(0);
  }
  b_in1.set_size(i);
  stride_0_0 = (in1.size(0) != 1);
  stride_1_0 = (in3.size(0) != 1);
  if (in3.size(0) == 1) {
    loop_ub = in1.size(0);
  } else {
    loop_ub = in3.size(0);
  }
  for (i = 0; i < loop_ub; i++) {
    b_in1[i] = in1[i * stride_0_0] - in3[i * stride_1_0];
  }
  in1.set_size(b_in1.size(0));
  loop_ub = b_in1.size(0);
  for (i = 0; i < loop_ub; i++) {
    in1[i] = b_in1[i];
  }
  loop_ub = in1.size(0);
  for (i = 0; i < loop_ub; i++) {
    double varargin_1;
    varargin_1 = in1[i];
    in1[i] = std::fmax(varargin_1, in4);
  }
}

//
// File trailer for unsafeSxfun.cpp
//
// [EOF]
//
