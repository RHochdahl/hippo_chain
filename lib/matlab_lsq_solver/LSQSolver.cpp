//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: LSQSolver.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "LSQSolver.h"
#include "driver.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "solveLSQ_internal_types.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
LSQSolver::LSQSolver() = default;

//
// Arguments    : void
// Return Type  : void
//
LSQSolver::~LSQSolver() = default;

//
// Arguments    : const coder::array<double, 2U> &A
//                const coder::array<double, 1U> &b
//                const coder::array<double, 1U> &x0
//                coder::array<double, 1U> &x
// Return Type  : void
//
void LSQSolver::solveLSQ(const coder::array<double, 2U> &A,
                         const coder::array<double, 1U> &b,
                         const coder::array<double, 1U> &x0,
                         coder::array<double, 1U> &x)
{
  coder::array<double, 2U> H;
  coder::array<double, 1U> f;
  b_struct_T QPObjective;
  c_struct_T QRManager;
  d_struct_T CholRegManager;
  e_struct_T memspace;
  f_struct_T WorkingSet;
  struct_T solution;
  double f_infnrm;
  double temp;
  int C_cols;
  int C_rows;
  int ar;
  int br;
  int i;
  int i1;
  int ic;
  int lastColC;
  int maxDims;
  int nVar;
  int w;
  nVar = x0.size(0) - 1;
  C_rows = A.size(0);
  C_cols = A.size(1);
  H.set_size(A.size(1), A.size(1));
  lastColC = A.size(1) * (A.size(1) - 1);
  for (maxDims = 0; C_cols < 0 ? maxDims >= lastColC : maxDims <= lastColC;
       maxDims += C_cols) {
    i = maxDims + 1;
    i1 = maxDims + C_cols;
    for (ic = i; ic <= i1; ic++) {
      H[ic - 1] = 0.0;
    }
  }
  br = -1;
  for (maxDims = 0; C_cols < 0 ? maxDims >= lastColC : maxDims <= lastColC;
       maxDims += C_cols) {
    ar = -1;
    i = maxDims + 1;
    i1 = maxDims + C_cols;
    for (ic = i; ic <= i1; ic++) {
      temp = 0.0;
      for (w = 0; w < C_rows; w++) {
        temp += A[(w + ar) + 1] * A[(w + br) + 1];
      }
      H[ic - 1] = H[ic - 1] + temp;
      ar += C_rows;
    }
    br += C_rows;
  }
  f.set_size(A.size(1));
  if (A.size(0) != 0) {
    C_cols = A.size(1);
    for (br = 0; br < C_cols; br++) {
      f[br] = 0.0;
    }
    br = 0;
    i = A.size(0) * (A.size(1) - 1) + 1;
    for (C_cols = 1; C_rows < 0 ? C_cols >= i : C_cols <= i; C_cols += C_rows) {
      temp = 0.0;
      i1 = (C_cols + C_rows) - 1;
      for (lastColC = C_cols; lastColC <= i1; lastColC++) {
        temp += A[lastColC - 1] * b[lastColC - C_cols];
      }
      f[br] = f[br] + -temp;
      br++;
    }
  }
  ar = (A.size(1) + A.size(1)) + 1;
  solution.xstar.set_size(x0.size(0) + 1);
  solution.fstar = 0.0;
  solution.firstorderopt = 0.0;
  solution.lambda.set_size(ar);
  for (i = 0; i < ar; i++) {
    solution.lambda[i] = 0.0;
  }
  solution.state = 0;
  solution.maxConstr = 0.0;
  solution.iterations = 0;
  solution.searchDir.set_size(x0.size(0) + 1);
  C_cols = x0.size(0);
  for (i = 0; i <= C_cols; i++) {
    solution.searchDir[i] = 0.0;
  }
  for (lastColC = 0; lastColC <= nVar; lastColC++) {
    solution.xstar[lastColC] = x0[lastColC];
  }
  QPObjective.grad.set_size(x0.size(0) + 1);
  QPObjective.Hx.set_size(x0.size(0));
  QPObjective.maxVar = x0.size(0) + 1;
  QPObjective.beta = 0.0;
  QPObjective.rho = 0.0;
  QPObjective.prev_objtype = 3;
  QPObjective.prev_nvar = 0;
  QPObjective.prev_hasLinear = false;
  QPObjective.gammaScalar = 0.0;
  QPObjective.nvar = x0.size(0);
  QPObjective.hasLinear = true;
  QPObjective.objtype = 3;
  maxDims = x0.size(0) + 1;
  if (maxDims < ar) {
    maxDims = ar;
  }
  QRManager.ldq = x0.size(0) + 1;
  QRManager.QR.set_size(x0.size(0) + 1, maxDims);
  QRManager.Q.set_size(x0.size(0) + 1, x0.size(0) + 1);
  C_cols = (x0.size(0) + 1) * (x0.size(0) + 1);
  for (i = 0; i < C_cols; i++) {
    QRManager.Q[i] = 0.0;
  }
  QRManager.jpvt.set_size(maxDims);
  for (i = 0; i < maxDims; i++) {
    QRManager.jpvt[i] = 0;
  }
  QRManager.mrows = 0;
  QRManager.ncols = 0;
  C_cols = x0.size(0) + 1;
  if (C_cols > maxDims) {
    C_cols = maxDims;
  }
  QRManager.tau.set_size(C_cols);
  QRManager.minRowCol = 0;
  QRManager.usedPivoting = false;
  CholRegManager.FMat.set_size((x0.size(0) + 1) * (x0.size(0) + 1));
  CholRegManager.ldm = x0.size(0) + 1;
  CholRegManager.ndims = 0;
  CholRegManager.info = 0;
  CholRegManager.ConvexCheck = true;
  CholRegManager.regTol_ = 0.0;
  CholRegManager.workspace_.set_size(48 * (x0.size(0) + 1));
  CholRegManager.workspace2_.set_size(48 * (x0.size(0) + 1));
  CholRegManager.scaleFactor = 100.0;
  lastColC = x0.size(0);
  C_cols = x0.size(0) + 1;
  WorkingSet.mConstr = 0;
  WorkingSet.mConstrOrig = 0;
  WorkingSet.mConstrMax = ar;
  WorkingSet.nVar = lastColC;
  WorkingSet.nVarOrig = lastColC;
  WorkingSet.nVarMax = C_cols;
  WorkingSet.ldA = C_cols;
  WorkingSet.Aineq.size[0] = 0;
  WorkingSet.Aeq.size[0] = 0;
  WorkingSet.lb.set_size(C_cols);
  WorkingSet.ub.set_size(C_cols);
  WorkingSet.indexLB.set_size(C_cols);
  WorkingSet.indexUB.set_size(C_cols);
  WorkingSet.indexFixed.set_size(C_cols);
  WorkingSet.mEqRemoved = 0;
  WorkingSet.ATwset.set_size(C_cols * ar);
  WorkingSet.bwset.set_size(ar);
  WorkingSet.nActiveConstr = 0;
  WorkingSet.maxConstrWorkspace.set_size(ar);
  for (C_rows = 0; C_rows < 5; C_rows++) {
    WorkingSet.sizes[C_rows] = 0;
    WorkingSet.sizesNormal[C_rows] = 0;
    WorkingSet.sizesPhaseOne[C_rows] = 0;
    WorkingSet.sizesRegularized[C_rows] = 0;
    WorkingSet.sizesRegPhaseOne[C_rows] = 0;
  }
  for (C_rows = 0; C_rows < 6; C_rows++) {
    WorkingSet.isActiveIdx[C_rows] = 0;
    WorkingSet.isActiveIdxNormal[C_rows] = 0;
    WorkingSet.isActiveIdxPhaseOne[C_rows] = 0;
    WorkingSet.isActiveIdxRegularized[C_rows] = 0;
    WorkingSet.isActiveIdxRegPhaseOne[C_rows] = 0;
  }
  WorkingSet.isActiveConstr.set_size(ar);
  WorkingSet.Wid.set_size(ar);
  WorkingSet.Wlocalidx.set_size(ar);
  for (C_rows = 0; C_rows < 5; C_rows++) {
    WorkingSet.nWConstr[C_rows] = 0;
  }
  WorkingSet.probType = 3;
  WorkingSet.SLACK0 = 1.0E-5;
  lastColC = x0.size(0) - 1;
  ic = 0;
  w = 0;
  if (A.size(1) != 0) {
    for (br = 0; br <= lastColC; br++) {
      WorkingSet.indexLB[br] = br + 1;
      WorkingSet.indexUB[br] = br + 1;
    }
    w = x0.size(0);
    ic = x0.size(0);
  }
  C_cols = ic + w;
  WorkingSet.mConstr = C_cols;
  WorkingSet.mConstrOrig = C_cols;
  WorkingSet.mConstrMax = ar;
  WorkingSet.sizes[0] = 0;
  WorkingSet.sizes[1] = 0;
  WorkingSet.sizes[2] = 0;
  WorkingSet.sizes[3] = ic;
  WorkingSet.sizes[4] = w;
  WorkingSet.sizesPhaseOne[0] = 0;
  WorkingSet.sizesPhaseOne[1] = 0;
  WorkingSet.sizesPhaseOne[2] = 0;
  WorkingSet.sizesPhaseOne[3] = ic + 1;
  WorkingSet.sizesPhaseOne[4] = w;
  WorkingSet.isActiveIdx[0] = 1;
  WorkingSet.isActiveIdx[1] = 0;
  WorkingSet.isActiveIdx[2] = 0;
  WorkingSet.isActiveIdx[3] = 0;
  WorkingSet.isActiveIdx[4] = ic;
  WorkingSet.isActiveIdx[5] = w;
  for (lastColC = 0; lastColC < 5; lastColC++) {
    WorkingSet.sizesNormal[lastColC] = WorkingSet.sizes[lastColC];
    WorkingSet.sizesRegularized[lastColC] = WorkingSet.sizes[lastColC];
    WorkingSet.sizesRegPhaseOne[lastColC] = WorkingSet.sizesPhaseOne[lastColC];
    WorkingSet.isActiveIdx[lastColC + 1] += WorkingSet.isActiveIdx[lastColC];
  }
  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdxNormal[i] = WorkingSet.isActiveIdx[i];
  }
  WorkingSet.isActiveIdxPhaseOne[0] = 1;
  WorkingSet.isActiveIdxPhaseOne[1] = 0;
  WorkingSet.isActiveIdxPhaseOne[2] = 0;
  WorkingSet.isActiveIdxPhaseOne[3] = 0;
  WorkingSet.isActiveIdxPhaseOne[4] = ic + 1;
  WorkingSet.isActiveIdxPhaseOne[5] = w;
  for (lastColC = 0; lastColC < 5; lastColC++) {
    WorkingSet.isActiveIdxPhaseOne[lastColC + 1] +=
        WorkingSet.isActiveIdxPhaseOne[lastColC];
  }
  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdxRegularized[i] = WorkingSet.isActiveIdx[i];
    WorkingSet.isActiveIdxRegPhaseOne[i] = WorkingSet.isActiveIdxPhaseOne[i];
  }
  if (A.size(1) != 0) {
    i = WorkingSet.nVar;
    for (br = 0; br < i; br++) {
      WorkingSet.lb[br] = 1.0;
    }
    C_cols = WorkingSet.nVar;
    for (lastColC = 0; lastColC < C_cols; lastColC++) {
      WorkingSet.ub[lastColC] = 1.0;
    }
  }
  coder::optim::coder::qpactiveset::WorkingSet::setProblemType(&WorkingSet, 3);
  C_cols = WorkingSet.isActiveIdx[2];
  i = WorkingSet.mConstrMax;
  for (br = C_cols; br <= i; br++) {
    WorkingSet.isActiveConstr[br - 1] = false;
  }
  WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
  WorkingSet.nWConstr[1] = 0;
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
  C_cols = WorkingSet.sizes[0];
  for (br = 0; br < C_cols; br++) {
    WorkingSet.Wid[br] = 1;
    WorkingSet.Wlocalidx[br] = br + 1;
    WorkingSet.isActiveConstr[br] = true;
    lastColC = WorkingSet.ldA * br;
    i = WorkingSet.indexFixed[br];
    for (C_rows = 0; C_rows <= i - 2; C_rows++) {
      WorkingSet.ATwset[C_rows + lastColC] = 0.0;
    }
    WorkingSet.ATwset[(WorkingSet.indexFixed[br] + lastColC) - 1] = 1.0;
    i = WorkingSet.indexFixed[br] + 1;
    i1 = WorkingSet.nVar;
    for (C_rows = i; C_rows <= i1; C_rows++) {
      WorkingSet.ATwset[(C_rows + lastColC) - 1] = 0.0;
    }
    WorkingSet.bwset[br] = 1.0;
  }
  WorkingSet.SLACK0 = 0.0;
  C_cols = x0.size(0) + 1;
  if (C_cols < 2) {
    C_cols = 2;
  }
  memspace.workspace_double.set_size(maxDims, C_cols);
  memspace.workspace_int.set_size(maxDims);
  memspace.workspace_sort.set_size(maxDims);
  temp = 0.0;
  f_infnrm = 0.0;
  i = H.size(1);
  i1 = H.size(0);
  for (C_cols = 0; C_cols < i; C_cols++) {
    double colSum;
    colSum = 0.0;
    for (lastColC = 0; lastColC < i1; lastColC++) {
      colSum += std::abs(H[lastColC + H.size(0) * C_cols]);
    }
    temp = std::fmax(temp, colSum);
    f_infnrm = std::fmax(f_infnrm, std::abs(f[C_cols]));
  }
  coder::optim::coder::qpactiveset::driver(
      H, f, &solution, &memspace, &WorkingSet, &QRManager, &CholRegManager,
      &QPObjective, 10 * ((x0.size(0) + ic) + w),
      std::fmax(std::fmax(1.0, f_infnrm), temp));
  x.set_size(x0.size(0));
  for (lastColC = 0; lastColC <= nVar; lastColC++) {
    x[lastColC] = solution.xstar[lastColC];
  }
}

//
// File trailer for LSQSolver.cpp
//
// [EOF]
//
