//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RemoveDependentEq_.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "RemoveDependentEq_.h"
#include "computeQ_.h"
#include "countsort.h"
#include "factorQRE.h"
#include "rt_nonfinite.h"
#include "solveLSQ_internal_types.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : e_struct_T *memspace
//                f_struct_T *workingset
//                c_struct_T *qrmanager
// Return Type  : int
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
int RemoveDependentEq_(e_struct_T *memspace, f_struct_T *workingset,
                       c_struct_T *qrmanager)
{
  int mTotalWorkingEq_tmp_tmp;
  int mWorkingFixed;
  int nDepInd;
  int nVar;
  nVar = workingset->nVar - 1;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    double tol;
    int idx_col;
    int totalRank;
    for (totalRank = 0; totalRank < mTotalWorkingEq_tmp_tmp; totalRank++) {
      for (idx_col = 0; idx_col <= nVar; idx_col++) {
        qrmanager->QR[totalRank + qrmanager->ldq * idx_col] =
            workingset->ATwset[idx_col + workingset->ldA * totalRank];
      }
    }
    nDepInd = mTotalWorkingEq_tmp_tmp - workingset->nVar;
    if (nDepInd <= 0) {
      nDepInd = 0;
    }
    for (idx_col = 0; idx_col <= nVar; idx_col++) {
      qrmanager->jpvt[idx_col] = 0;
    }
    QRManager::factorQRE(qrmanager, mTotalWorkingEq_tmp_tmp, workingset->nVar);
    tol =
        100.0 * static_cast<double>(workingset->nVar) * 2.2204460492503131E-16;
    totalRank = workingset->nVar;
    if (totalRank > mTotalWorkingEq_tmp_tmp) {
      totalRank = mTotalWorkingEq_tmp_tmp;
    }
    totalRank += qrmanager->ldq * (totalRank - 1);
    while ((totalRank > 0) && (std::abs(qrmanager->QR[totalRank - 1]) < tol)) {
      totalRank = (totalRank - qrmanager->ldq) - 1;
      nDepInd++;
    }
    if (nDepInd > 0) {
      bool exitg1;
      QRManager::computeQ_(qrmanager, qrmanager->mrows);
      idx_col = 0;
      exitg1 = false;
      while ((!exitg1) && (idx_col <= nDepInd - 1)) {
        double qtb;
        totalRank = qrmanager->ldq * ((mTotalWorkingEq_tmp_tmp - idx_col) - 1);
        qtb = 0.0;
        for (int k{0}; k < mTotalWorkingEq_tmp_tmp; k++) {
          qtb += qrmanager->Q[totalRank + k] * workingset->bwset[k];
        }
        if (std::abs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idx_col++;
        }
      }
    }
    if (nDepInd > 0) {
      int ix0;
      for (idx_col = 0; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
        totalRank = qrmanager->ldq * idx_col;
        ix0 = workingset->ldA * idx_col;
        for (int k{0}; k <= nVar; k++) {
          qrmanager->QR[totalRank + k] = workingset->ATwset[ix0 + k];
        }
      }
      for (idx_col = 0; idx_col < mWorkingFixed; idx_col++) {
        qrmanager->jpvt[idx_col] = 1;
      }
      ix0 = workingset->nWConstr[0] + 1;
      for (idx_col = ix0; idx_col <= mTotalWorkingEq_tmp_tmp; idx_col++) {
        qrmanager->jpvt[idx_col - 1] = 0;
      }
      QRManager::factorQRE(qrmanager, workingset->nVar,
                           mTotalWorkingEq_tmp_tmp);
      for (idx_col = 0; idx_col < nDepInd; idx_col++) {
        memspace->workspace_int[idx_col] =
            qrmanager->jpvt[(mTotalWorkingEq_tmp_tmp - nDepInd) + idx_col];
      }
      utils::countsort(memspace->workspace_int, nDepInd,
                       memspace->workspace_sort, 1, mTotalWorkingEq_tmp_tmp);
      for (idx_col = nDepInd; idx_col >= 1; idx_col--) {
        ix0 = workingset->nWConstr[0] + workingset->nWConstr[1];
        if (ix0 != 0) {
          totalRank = memspace->workspace_int[idx_col - 1];
          if (totalRank <= ix0) {
            if ((workingset->nActiveConstr == ix0) || (totalRank == ix0)) {
              workingset->mEqRemoved++;
              // A check that is always false is detected at compile-time.
              // Eliminating code that follows.
            } else {
              workingset->mEqRemoved++;
              // A check that is always false is detected at compile-time.
              // Eliminating code that follows.
            }
          }
        }
      }
    }
  }
  return nDepInd;
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for RemoveDependentEq_.cpp
//
// [EOF]
//
