//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: LSQSolver2.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 14-Mar-2023 23:05:22
//

#ifndef LSQSOLVER2_H
#define LSQSOLVER2_H

// Include Files
#include "../rtwtypes.h"
#include "../coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class LSQSolver2 {
public:
  LSQSolver2();
  ~LSQSolver2();
  void solveLSQ2(const coder::array<double, 2U> &Q,
                 const coder::array<double, 1U> &q,
                 const coder::array<double, 1U> &x0,
                 coder::array<double, 1U> &x);
};

#endif
//
// File trailer for LSQSolver2.h
//
// [EOF]
//
