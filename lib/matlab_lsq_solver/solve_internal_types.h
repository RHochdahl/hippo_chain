//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: solve_internal_types.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 02-Jan-2023 15:29:42
//

#ifndef SOLVE_INTERNAL_TYPES_H
#define SOLVE_INTERNAL_TYPES_H

// Include Files
#include "solve_types.h"
#include "rtwtypes.h"
#include "coder_array.h"

// Type Definitions
struct struct_T {
  coder::array<double, 1U> xstar;
  double fstar;
  double firstorderopt;
  coder::array<double, 1U> lambda;
  int state;
  double maxConstr;
  int iterations;
  coder::array<double, 1U> searchDir;
};

struct b_struct_T {
  coder::array<double, 1U> grad;
  coder::array<double, 1U> Hx;
  boolean_T hasLinear;
  int nvar;
  int maxVar;
  double beta;
  double rho;
  int objtype;
  int prev_objtype;
  int prev_nvar;
  boolean_T prev_hasLinear;
  double gammaScalar;
};

struct c_struct_T {
  int ldq;
  coder::array<double, 2U> QR;
  coder::array<double, 2U> Q;
  coder::array<int, 1U> jpvt;
  int mrows;
  int ncols;
  coder::array<double, 1U> tau;
  int minRowCol;
  boolean_T usedPivoting;
};

struct d_struct_T {
  coder::array<double, 1U> FMat;
  int ldm;
  int ndims;
  int info;
  double scaleFactor;
  boolean_T ConvexCheck;
  double regTol_;
  coder::array<double, 1U> workspace_;
  coder::array<double, 1U> workspace2_;
};

struct e_struct_T {
  coder::array<double, 2U> workspace_double;
  coder::array<int, 1U> workspace_int;
  coder::array<int, 1U> workspace_sort;
};

struct emxArray_real_T_0 {
  int size[1];
};

struct f_struct_T {
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  emxArray_real_T_0 Aineq;
  emxArray_real_T_0 Aeq;
  coder::array<double, 1U> lb;
  coder::array<double, 1U> ub;
  coder::array<int, 1U> indexLB;
  coder::array<int, 1U> indexUB;
  coder::array<int, 1U> indexFixed;
  int mEqRemoved;
  coder::array<double, 1U> ATwset;
  coder::array<double, 1U> bwset;
  int nActiveConstr;
  coder::array<double, 1U> maxConstrWorkspace;
  int sizes[5];
  int sizesNormal[5];
  int sizesPhaseOne[5];
  int sizesRegularized[5];
  int sizesRegPhaseOne[5];
  int isActiveIdx[6];
  int isActiveIdxNormal[6];
  int isActiveIdxPhaseOne[6];
  int isActiveIdxRegularized[6];
  int isActiveIdxRegPhaseOne[6];
  coder::array<boolean_T, 1U> isActiveConstr;
  coder::array<int, 1U> Wid;
  coder::array<int, 1U> Wlocalidx;
  int nWConstr[5];
  int probType;
  double SLACK0;
};

#endif
//
// File trailer for solve_internal_types.h
//
// [EOF]
//
