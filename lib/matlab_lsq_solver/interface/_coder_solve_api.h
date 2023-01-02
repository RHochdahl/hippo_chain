//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_solve_api.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 02-Jan-2023 15:29:42
//

#ifndef _CODER_SOLVE_API_H
#define _CODER_SOLVE_API_H

// Include Files
#include "coder_array_mex.h"
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void solve(coder::array<real_T, 2U> *A, coder::array<real_T, 1U> *b,
              coder::array<real_T, 1U> *x);

void solve_api(const mxArray *const prhs[2], const mxArray **plhs);

void solve_atexit();

void solve_initialize();

void solve_terminate();

void solve_xil_shutdown();

void solve_xil_terminate();

#endif
//
// File trailer for _coder_solve_api.h
//
// [EOF]
//
