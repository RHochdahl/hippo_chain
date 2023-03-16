//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: feasibleX0ForWorkingSet.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "feasibleX0ForWorkingSet.h"
#include "computeQ_.h"
#include "factorQR.h"
#include "../rt_nonfinite.h"
#include "solveLSQ_internal_types.h"
#include "xzgeqp3.h"
#include "../coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : ::coder::array<double, 2U> &workspace
//                ::coder::array<double, 1U> &xCurrent
//                const f_struct_T *workingset
//                c_struct_T *qrmanager
// Return Type  : bool
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
bool feasibleX0ForWorkingSet(::coder::array<double, 2U> &workspace,
                             ::coder::array<double, 1U> &xCurrent,
                             const f_struct_T *workingset,
                             c_struct_T *qrmanager)
{
  array<double, 2U> B;
  int mWConstr;
  int nVar;
  bool nonDegenerateWset;
  mWConstr = workingset->nActiveConstr;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  if (mWConstr != 0) {
    double c;
    int ar;
    int i;
    int i1;
    int iAcol;
    int idx;
    int ix0_2;
    int iy;
    int jBcol;
    for (idx = 0; idx < mWConstr; idx++) {
      workspace[idx] = workingset->bwset[idx];
      workspace[idx + workspace.size(0)] = workingset->bwset[idx];
    }
    iAcol = workingset->ldA;
    if (mWConstr != 0) {
      iy = 0;
      i = workingset->ldA * (mWConstr - 1) + 1;
      for (jBcol = 1; iAcol < 0 ? jBcol >= i : jBcol <= i; jBcol += iAcol) {
        c = 0.0;
        i1 = (jBcol + nVar) - 1;
        for (ix0_2 = jBcol; ix0_2 <= i1; ix0_2++) {
          c += workingset->ATwset[ix0_2 - 1] * xCurrent[ix0_2 - jBcol];
        }
        workspace[iy] = workspace[iy] + -c;
        iy++;
      }
    }
    if (mWConstr >= nVar) {
      int ldq;
      int ldw;
      qrmanager->usedPivoting = false;
      qrmanager->mrows = mWConstr;
      qrmanager->ncols = nVar;
      for (ar = 0; ar < nVar; ar++) {
        iAcol = qrmanager->ldq * ar;
        for (jBcol = 0; jBcol < mWConstr; jBcol++) {
          qrmanager->QR[jBcol + iAcol] =
              workingset->ATwset[ar + workingset->ldA * jBcol];
        }
        qrmanager->jpvt[ar] = ar + 1;
      }
      if (mWConstr <= nVar) {
        i = mWConstr;
      } else {
        i = nVar;
      }
      qrmanager->minRowCol = i;
      B.set_size(qrmanager->QR.size(0), qrmanager->QR.size(1));
      iAcol = qrmanager->QR.size(0) * qrmanager->QR.size(1);
      for (i1 = 0; i1 < iAcol; i1++) {
        B[i1] = qrmanager->QR[i1];
      }
      iAcol = qrmanager->QR.size(0);
      iy = qrmanager->QR.size(1);
      if (iAcol <= iy) {
        iy = iAcol;
      }
      qrmanager->tau.set_size(iy);
      for (i1 = 0; i1 < iy; i1++) {
        qrmanager->tau[i1] = 0.0;
      }
      if (i >= 1) {
        internal::reflapack::qrf(B, mWConstr, nVar, i, qrmanager->tau);
      }
      qrmanager->QR.set_size(B.size(0), B.size(1));
      iAcol = B.size(0) * B.size(1);
      for (i = 0; i < iAcol; i++) {
        qrmanager->QR[i] = B[i];
      }
      QRManager::computeQ_(qrmanager, mWConstr);
      ldq = qrmanager->ldq;
      ldw = workspace.size(0);
      B.set_size(workspace.size(0), workspace.size(1));
      iAcol = workspace.size(0) * workspace.size(1);
      for (i = 0; i < iAcol; i++) {
        B[i] = workspace[i];
      }
      for (jBcol = 0; ldw < 0 ? jBcol >= ldw : jBcol <= ldw; jBcol += ldw) {
        i = jBcol + 1;
        i1 = jBcol + nVar;
        for (int ic{i}; ic <= i1; ic++) {
          workspace[ic - 1] = 0.0;
        }
      }
      iAcol = -1;
      for (jBcol = 0; ldw < 0 ? jBcol >= ldw : jBcol <= ldw; jBcol += ldw) {
        ar = -1;
        i = jBcol + 1;
        i1 = jBcol + nVar;
        for (int ic{i}; ic <= i1; ic++) {
          c = 0.0;
          for (iy = 0; iy < mWConstr; iy++) {
            c += qrmanager->Q[(iy + ar) + 1] * B[(iy + iAcol) + 1];
          }
          workspace[ic - 1] = workspace[ic - 1] + c;
          ar += ldq;
        }
        iAcol += ldw;
      }
      for (iy = 0; iy < 2; iy++) {
        jBcol = ldw * iy - 1;
        for (idx = nVar; idx >= 1; idx--) {
          ar = ldq * (idx - 1) - 1;
          i = idx + jBcol;
          if (workspace[i] != 0.0) {
            workspace[i] = workspace[i] / qrmanager->QR[idx + ar];
            for (ix0_2 = 0; ix0_2 <= idx - 2; ix0_2++) {
              i1 = (ix0_2 + jBcol) + 1;
              workspace[i1] = workspace[i1] -
                              workspace[i] * qrmanager->QR[(ix0_2 + ar) + 1];
            }
          }
        }
      }
    } else {
      int ldq;
      int ldw;
      QRManager::factorQR(qrmanager, workingset->ATwset, nVar, mWConstr,
                          workingset->ldA);
      QRManager::computeQ_(qrmanager, qrmanager->minRowCol);
      ldq = qrmanager->ldq;
      ldw = workspace.size(0);
      for (iy = 0; iy < 2; iy++) {
        jBcol = ldw * iy;
        for (ix0_2 = 0; ix0_2 < mWConstr; ix0_2++) {
          iAcol = ldq * ix0_2;
          ar = ix0_2 + jBcol;
          c = workspace[ar];
          for (idx = 0; idx < ix0_2; idx++) {
            c -= qrmanager->QR[idx + iAcol] * workspace[idx + jBcol];
          }
          workspace[ar] = c / qrmanager->QR[ix0_2 + iAcol];
        }
      }
      B.set_size(workspace.size(0), workspace.size(1));
      iAcol = workspace.size(0) * workspace.size(1);
      for (i = 0; i < iAcol; i++) {
        B[i] = workspace[i];
      }
      for (jBcol = 0; ldw < 0 ? jBcol >= ldw : jBcol <= ldw; jBcol += ldw) {
        i = jBcol + 1;
        i1 = jBcol + nVar;
        for (int ic{i}; ic <= i1; ic++) {
          workspace[ic - 1] = 0.0;
        }
      }
      iAcol = 0;
      for (jBcol = 0; ldw < 0 ? jBcol >= ldw : jBcol <= ldw; jBcol += ldw) {
        ar = -1;
        i = iAcol + 1;
        i1 = iAcol + mWConstr;
        for (idx = i; idx <= i1; idx++) {
          iy = jBcol + 1;
          ix0_2 = jBcol + nVar;
          for (int ic{iy}; ic <= ix0_2; ic++) {
            workspace[ic - 1] = workspace[ic - 1] +
                                B[idx - 1] * qrmanager->Q[(ar + ic) - jBcol];
          }
          ar += ldq;
        }
        iAcol += ldw;
      }
    }
    idx = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (idx <= nVar - 1) {
        c = workspace[idx];
        if (std::isinf(c) || std::isnan(c)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          c = workspace[idx + workspace.size(0)];
          if (std::isinf(c) || std::isnan(c)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            idx++;
          }
        }
      } else {
        double constrViolation_basicX;
        iAcol = nVar - 1;
        for (idx = 0; idx <= iAcol; idx++) {
          workspace[idx] = workspace[idx] + xCurrent[idx];
        }
        ix0_2 = workspace.size(0) - 1;
        iAcol = workingset->sizes[3];
        iy = workingset->sizes[4];
        ar = workingset->sizes[0];
        c = 0.0;
        if (workingset->sizes[3] > 0) {
          for (idx = 0; idx < iAcol; idx++) {
            jBcol = workingset->indexLB[idx] - 1;
            c = std::fmax(c, -workspace[jBcol] - workingset->lb[jBcol]);
          }
        }
        if (workingset->sizes[4] > 0) {
          for (idx = 0; idx < iy; idx++) {
            c = std::fmax(c, workspace[workingset->indexUB[idx] - 1] - 1.0);
          }
        }
        if (workingset->sizes[0] > 0) {
          for (idx = 0; idx < ar; idx++) {
            c = std::fmax(
                c, std::abs(workspace[workingset->indexFixed[idx] - 1] - 1.0));
          }
        }
        iAcol = workingset->sizes[3];
        iy = workingset->sizes[4];
        ar = workingset->sizes[0];
        constrViolation_basicX = 0.0;
        if (workingset->sizes[3] > 0) {
          for (idx = 0; idx < iAcol; idx++) {
            constrViolation_basicX =
                std::fmax(constrViolation_basicX,
                          -workspace[ix0_2 + workingset->indexLB[idx]] -
                              workingset->lb[workingset->indexLB[idx] - 1]);
          }
        }
        if (workingset->sizes[4] > 0) {
          for (idx = 0; idx < iy; idx++) {
            constrViolation_basicX =
                std::fmax(constrViolation_basicX,
                          workspace[ix0_2 + workingset->indexUB[idx]] - 1.0);
          }
        }
        if (workingset->sizes[0] > 0) {
          for (idx = 0; idx < ar; idx++) {
            constrViolation_basicX = std::fmax(
                constrViolation_basicX,
                std::abs(workspace[ix0_2 + workingset->indexFixed[idx]] - 1.0));
          }
        }
        if ((c <= 2.2204460492503131E-16) || (c < constrViolation_basicX)) {
          for (idx = 0; idx < nVar; idx++) {
            xCurrent[idx] = workspace[idx];
          }
        } else {
          for (idx = 0; idx < nVar; idx++) {
            xCurrent[idx] = workspace[(ix0_2 + idx) + 1];
          }
        }
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return nonDegenerateWset;
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for feasibleX0ForWorkingSet.cpp
//
// [EOF]
//
