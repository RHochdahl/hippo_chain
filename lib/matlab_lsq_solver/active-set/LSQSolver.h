//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: LSQSolver.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

#ifndef LSQSOLVER_H
#define LSQSOLVER_H

// Include Files
#include "../rtwtypes.h"
#include "../coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class LSQSolver {
public:
  LSQSolver();
  ~LSQSolver();
  void solveLSQ(const coder::array<double, 2U> &A,
                const coder::array<double, 1U> &b,
                const coder::array<double, 1U> &x0,
                coder::array<double, 1U> &x);
};

#endif
//
// File trailer for LSQSolver.h
//
// [EOF]
//
