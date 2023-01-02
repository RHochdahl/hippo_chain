//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: LSQSolver.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 02-Jan-2023 15:29:42
//

#ifndef LSQSOLVER_H
#define LSQSOLVER_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class LSQSolver {
public:
  LSQSolver();
  ~LSQSolver();
  void solve(const coder::array<double, 2U> &A,
                const coder::array<double, 1U> &b, coder::array<double, 1U> &x);
};

#endif
//
// File trailer for LSQSolver.h
//
// [EOF]
//
