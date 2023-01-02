//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: LSQSolver.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 02-Jan-2023 15:29:42
//

// Include Files
#include "LSQSolver.h"
#include "driver.h"
#include "solve_internal_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
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
//                coder::array<double, 1U> &x
// Return Type  : void
//
void LSQSolver::solve(const coder::array<double, 2U> &A,
                               const coder::array<double, 1U> &b,
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
  int b_i;
  int br;
  int i1;
  int lastColC;
  int maxDims;
  int nVar;
  int w;
  nVar = A.size(1) - 1;
  C_rows = A.size(0);
  C_cols = A.size(1);
  H.set_size(A.size(1), A.size(1));
  lastColC = A.size(1) * (A.size(1) - 1);
  for (int i{0}; C_cols < 0 ? i >= lastColC : i <= lastColC; i += C_cols) {
    b_i = i + 1;
    i1 = i + C_cols;
    for (maxDims = b_i; maxDims <= i1; maxDims++) {
      H[maxDims - 1] = 0.0;
    }
  }
  br = -1;
  for (int i{0}; C_cols < 0 ? i >= lastColC : i <= lastColC; i += C_cols) {
    ar = -1;
    b_i = i + 1;
    i1 = i + C_cols;
    for (maxDims = b_i; maxDims <= i1; maxDims++) {
      temp = 0.0;
      for (w = 0; w < C_rows; w++) {
        temp += A[(w + ar) + 1] * A[(w + br) + 1];
      }
      H[maxDims - 1] = H[maxDims - 1] + temp;
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
    b_i = A.size(0) * (A.size(1) - 1) + 1;
    for (C_cols = 1; C_rows < 0 ? C_cols >= b_i : C_cols <= b_i;
         C_cols += C_rows) {
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
  solution.xstar.set_size(A.size(1) + 1);
  solution.fstar = 0.0;
  solution.firstorderopt = 0.0;
  solution.lambda.set_size(ar);
  for (b_i = 0; b_i < ar; b_i++) {
    solution.lambda[b_i] = 0.0;
  }
  solution.state = 0;
  solution.maxConstr = 0.0;
  solution.iterations = 0;
  solution.searchDir.set_size(A.size(1) + 1);
  C_cols = A.size(1);
  for (b_i = 0; b_i <= C_cols; b_i++) {
    solution.searchDir[b_i] = 0.0;
  }
  for (lastColC = 0; lastColC <= nVar; lastColC++) {
    solution.xstar[lastColC] = 0.0;
  }
  QPObjective.grad.set_size(A.size(1) + 1);
  QPObjective.Hx.set_size(A.size(1));
  QPObjective.maxVar = A.size(1) + 1;
  QPObjective.beta = 0.0;
  QPObjective.rho = 0.0;
  QPObjective.prev_objtype = 3;
  QPObjective.prev_nvar = 0;
  QPObjective.prev_hasLinear = false;
  QPObjective.gammaScalar = 0.0;
  QPObjective.nvar = A.size(1);
  QPObjective.hasLinear = true;
  QPObjective.objtype = 3;
  maxDims = A.size(1) + 1;
  if (maxDims < ar) {
    maxDims = ar;
  }
  QRManager.ldq = A.size(1) + 1;
  QRManager.QR.set_size(A.size(1) + 1, maxDims);
  QRManager.Q.set_size(A.size(1) + 1, A.size(1) + 1);
  C_cols = (A.size(1) + 1) * (A.size(1) + 1);
  for (b_i = 0; b_i < C_cols; b_i++) {
    QRManager.Q[b_i] = 0.0;
  }
  QRManager.jpvt.set_size(maxDims);
  for (b_i = 0; b_i < maxDims; b_i++) {
    QRManager.jpvt[b_i] = 0;
  }
  QRManager.mrows = 0;
  QRManager.ncols = 0;
  C_cols = A.size(1) + 1;
  if (C_cols > maxDims) {
    C_cols = maxDims;
  }
  QRManager.tau.set_size(C_cols);
  QRManager.minRowCol = 0;
  QRManager.usedPivoting = false;
  CholRegManager.FMat.set_size((A.size(1) + 1) * (A.size(1) + 1));
  CholRegManager.ldm = A.size(1) + 1;
  CholRegManager.ndims = 0;
  CholRegManager.info = 0;
  CholRegManager.ConvexCheck = true;
  CholRegManager.regTol_ = 0.0;
  CholRegManager.workspace_.set_size(48 * (A.size(1) + 1));
  CholRegManager.workspace2_.set_size(48 * (A.size(1) + 1));
  CholRegManager.scaleFactor = 100.0;
  lastColC = A.size(1);
  C_cols = A.size(1) + 1;
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
  for (int i{0}; i < 5; i++) {
    WorkingSet.sizes[i] = 0;
    WorkingSet.sizesNormal[i] = 0;
    WorkingSet.sizesPhaseOne[i] = 0;
    WorkingSet.sizesRegularized[i] = 0;
    WorkingSet.sizesRegPhaseOne[i] = 0;
  }
  for (int i{0}; i < 6; i++) {
    WorkingSet.isActiveIdx[i] = 0;
    WorkingSet.isActiveIdxNormal[i] = 0;
    WorkingSet.isActiveIdxPhaseOne[i] = 0;
    WorkingSet.isActiveIdxRegularized[i] = 0;
    WorkingSet.isActiveIdxRegPhaseOne[i] = 0;
  }
  WorkingSet.isActiveConstr.set_size(ar);
  WorkingSet.Wid.set_size(ar);
  WorkingSet.Wlocalidx.set_size(ar);
  for (int i{0}; i < 5; i++) {
    WorkingSet.nWConstr[i] = 0;
  }
  WorkingSet.probType = 3;
  WorkingSet.SLACK0 = 1.0E-5;
  lastColC = A.size(1) - 1;
  w = 0;
  C_rows = 0;
  if (A.size(1) != 0) {
    for (br = 0; br <= lastColC; br++) {
      WorkingSet.indexLB[br] = br + 1;
      WorkingSet.indexUB[br] = br + 1;
    }
    C_rows = A.size(1);
    w = A.size(1);
  }
  C_cols = w + C_rows;
  WorkingSet.mConstr = C_cols;
  WorkingSet.mConstrOrig = C_cols;
  WorkingSet.mConstrMax = ar;
  WorkingSet.sizes[0] = 0;
  WorkingSet.sizes[1] = 0;
  WorkingSet.sizes[2] = 0;
  WorkingSet.sizes[3] = w;
  WorkingSet.sizes[4] = C_rows;
  WorkingSet.sizesPhaseOne[0] = 0;
  WorkingSet.sizesPhaseOne[1] = 0;
  WorkingSet.sizesPhaseOne[2] = 0;
  WorkingSet.sizesPhaseOne[3] = w + 1;
  WorkingSet.sizesPhaseOne[4] = C_rows;
  WorkingSet.isActiveIdx[0] = 1;
  WorkingSet.isActiveIdx[1] = 0;
  WorkingSet.isActiveIdx[2] = 0;
  WorkingSet.isActiveIdx[3] = 0;
  WorkingSet.isActiveIdx[4] = w;
  WorkingSet.isActiveIdx[5] = C_rows;
  for (lastColC = 0; lastColC < 5; lastColC++) {
    WorkingSet.sizesNormal[lastColC] = WorkingSet.sizes[lastColC];
    WorkingSet.sizesRegularized[lastColC] = WorkingSet.sizes[lastColC];
    WorkingSet.sizesRegPhaseOne[lastColC] = WorkingSet.sizesPhaseOne[lastColC];
    WorkingSet.isActiveIdx[lastColC + 1] += WorkingSet.isActiveIdx[lastColC];
  }
  for (b_i = 0; b_i < 6; b_i++) {
    WorkingSet.isActiveIdxNormal[b_i] = WorkingSet.isActiveIdx[b_i];
  }
  WorkingSet.isActiveIdxPhaseOne[0] = 1;
  WorkingSet.isActiveIdxPhaseOne[1] = 0;
  WorkingSet.isActiveIdxPhaseOne[2] = 0;
  WorkingSet.isActiveIdxPhaseOne[3] = 0;
  WorkingSet.isActiveIdxPhaseOne[4] = w + 1;
  WorkingSet.isActiveIdxPhaseOne[5] = C_rows;
  for (lastColC = 0; lastColC < 5; lastColC++) {
    WorkingSet.isActiveIdxPhaseOne[lastColC + 1] +=
        WorkingSet.isActiveIdxPhaseOne[lastColC];
  }
  for (b_i = 0; b_i < 6; b_i++) {
    WorkingSet.isActiveIdxRegularized[b_i] = WorkingSet.isActiveIdx[b_i];
    WorkingSet.isActiveIdxRegPhaseOne[b_i] =
        WorkingSet.isActiveIdxPhaseOne[b_i];
  }
  if (A.size(1) != 0) {
    b_i = WorkingSet.nVar;
    for (br = 0; br < b_i; br++) {
      WorkingSet.lb[br] = 1.0;
    }
    C_cols = WorkingSet.nVar;
    for (lastColC = 0; lastColC < C_cols; lastColC++) {
      WorkingSet.ub[lastColC] = 1.0;
    }
  }
  coder::optim::coder::qpactiveset::WorkingSet::setProblemType(&WorkingSet, 3);
  C_cols = WorkingSet.isActiveIdx[2];
  b_i = WorkingSet.mConstrMax;
  for (br = C_cols; br <= b_i; br++) {
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
    b_i = WorkingSet.indexFixed[br];
    for (int i{0}; i <= b_i - 2; i++) {
      WorkingSet.ATwset[i + lastColC] = 0.0;
    }
    WorkingSet.ATwset[(WorkingSet.indexFixed[br] + lastColC) - 1] = 1.0;
    b_i = WorkingSet.indexFixed[br] + 1;
    i1 = WorkingSet.nVar;
    for (int i{b_i}; i <= i1; i++) {
      WorkingSet.ATwset[(i + lastColC) - 1] = 0.0;
    }
    WorkingSet.bwset[br] = 1.0;
  }
  WorkingSet.SLACK0 = 0.0;
  C_cols = A.size(1) + 1;
  if (C_cols < 2) {
    C_cols = 2;
  }
  memspace.workspace_double.set_size(maxDims, C_cols);
  memspace.workspace_int.set_size(maxDims);
  memspace.workspace_sort.set_size(maxDims);
  temp = 0.0;
  f_infnrm = 0.0;
  b_i = H.size(1);
  i1 = H.size(0);
  for (C_cols = 0; C_cols < b_i; C_cols++) {
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
      &QPObjective, 10 * ((A.size(1) + w) + C_rows),
      std::fmax(std::fmax(1.0, f_infnrm), temp));
  x.set_size(A.size(1));
  for (lastColC = 0; lastColC <= nVar; lastColC++) {
    x[lastColC] = solution.xstar[lastColC];
  }
}

//
// File trailer for LSQSolver.cpp
//
// [EOF]
//
