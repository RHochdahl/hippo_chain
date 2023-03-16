//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: div.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 14-Mar-2023 23:05:22
//

#ifndef DIV_H
#define DIV_H

// Include Files
#include "../rtwtypes.h"
#include "../coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void binary_expand_op(coder::array<double, 1U> &in1,
                      const coder::array<double, 1U> &in2,
                      const coder::array<double, 1U> &in3,
                      const coder::array<double, 1U> &in4,
                      const coder::array<double, 1U> &in5);

void c_binary_expand_op(coder::array<double, 1U> &in1,
                        const coder::array<double, 1U> &in2,
                        const coder::array<double, 1U> &in3,
                        const coder::array<double, 1U> &in4,
                        const coder::array<double, 1U> &in5);

void rdivide(coder::array<double, 1U> &in1, const coder::array<double, 1U> &in2,
             const coder::array<double, 1U> &in3);

#endif
//
// File trailer for div.h
//
// [EOF]
//
