//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: compute_deltax.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "compute_deltax.h"
#include "fullColLDL2_.h"
#include "partialColLDL3_.h"
#include "rt_nonfinite.h"
#include "solveLSQ_internal_types.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &H
//                struct_T *solution
//                e_struct_T *memspace
//                const c_struct_T *qrmanager
//                d_struct_T *cholmanager
//                const b_struct_T *objective
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void compute_deltax(const ::coder::array<double, 2U> &H, struct_T *solution,
                    e_struct_T *memspace, const c_struct_T *qrmanager,
                    d_struct_T *cholmanager, const b_struct_T *objective)
{
  int mNull_tmp;
  int nVar_tmp;
  nVar_tmp = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    for (int idx{0}; idx <= nVar_tmp; idx++) {
      solution->searchDir[idx] = 0.0;
    }
  } else {
    int idx;
    for (idx = 0; idx <= nVar_tmp; idx++) {
      solution->searchDir[idx] = -objective->grad[idx];
    }
    if (qrmanager->ncols <= 0) {
      if (objective->objtype == 3) {
        double temp;
        int A_maxDiag_idx;
        int ldQ;
        int ldw;
        int nVars;
        temp = 1.4901161193847656E-6 * static_cast<double>(qrmanager->mrows);
        ldw = cholmanager->ldm + 1;
        cholmanager->ndims = qrmanager->mrows;
        for (idx = 0; idx <= nVar_tmp; idx++) {
          A_maxDiag_idx = (nVar_tmp + 1) * idx;
          nVars = cholmanager->ldm * idx;
          for (ldQ = 0; ldQ <= nVar_tmp; ldQ++) {
            cholmanager->FMat[nVars + ldQ] = H[A_maxDiag_idx + ldQ];
          }
        }
        if (qrmanager->mrows < 1) {
          A_maxDiag_idx = -1;
        } else {
          A_maxDiag_idx = 0;
          if (qrmanager->mrows > 1) {
            double smax;
            smax = std::abs(cholmanager->FMat[0]);
            for (ldQ = 2; ldQ <= nVar_tmp + 1; ldQ++) {
              double s;
              s = std::abs(cholmanager->FMat[(ldQ - 1) * ldw]);
              if (s > smax) {
                A_maxDiag_idx = ldQ - 1;
                smax = s;
              }
            }
          }
        }
        cholmanager->regTol_ = std::fmax(
            std::abs(cholmanager->FMat[A_maxDiag_idx +
                                       cholmanager->ldm * A_maxDiag_idx]) *
                2.2204460492503131E-16,
            std::abs(temp));
        if ((cholmanager->FMat.size(0) > 16384) && (qrmanager->mrows > 128)) {
          bool exitg1;
          ldQ = 0;
          exitg1 = false;
          while ((!exitg1) && (ldQ < nVar_tmp + 1)) {
            nVars = ldw * ldQ + 1;
            A_maxDiag_idx = (nVar_tmp - ldQ) + 1;
            if (ldQ + 48 <= nVar_tmp + 1) {
              DynamicRegCholManager::partialColLDL3_(cholmanager, nVars,
                                                     A_maxDiag_idx, temp);
              ldQ += 48;
            } else {
              DynamicRegCholManager::fullColLDL2_(cholmanager, nVars,
                                                  A_maxDiag_idx, temp);
              exitg1 = true;
            }
          }
        } else {
          DynamicRegCholManager::fullColLDL2_(cholmanager, 1, qrmanager->mrows,
                                              temp);
        }
        if (cholmanager->ConvexCheck) {
          idx = 0;
          int exitg2;
          do {
            exitg2 = 0;
            if (idx <= nVar_tmp) {
              if (cholmanager->FMat[idx + cholmanager->ldm * idx] <= 0.0) {
                cholmanager->info = -idx - 1;
                exitg2 = 1;
              } else {
                idx++;
              }
            } else {
              cholmanager->ConvexCheck = false;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          int i;
          nVars = cholmanager->ndims - 2;
          if (cholmanager->ndims != 0) {
            for (idx = 0; idx <= nVars + 1; idx++) {
              A_maxDiag_idx = idx + idx * cholmanager->ldm;
              i = nVars - idx;
              for (ldQ = 0; ldQ <= i; ldQ++) {
                ldw = (idx + ldQ) + 1;
                solution->searchDir[ldw] =
                    solution->searchDir[ldw] -
                    solution->searchDir[idx] *
                        cholmanager->FMat[(A_maxDiag_idx + ldQ) + 1];
              }
            }
          }
          i = cholmanager->ndims;
          for (idx = 0; idx < i; idx++) {
            solution->searchDir[idx] =
                solution->searchDir[idx] /
                cholmanager->FMat[idx + cholmanager->ldm * idx];
          }
          nVars = cholmanager->ndims;
          if (cholmanager->ndims != 0) {
            for (idx = nVars; idx >= 1; idx--) {
              A_maxDiag_idx = (idx - 1) * cholmanager->ldm;
              temp = solution->searchDir[idx - 1];
              i = idx + 1;
              for (ldQ = nVars; ldQ >= i; ldQ--) {
                temp -= cholmanager->FMat[(A_maxDiag_idx + ldQ) - 1] *
                        solution->searchDir[ldQ - 1];
              }
              solution->searchDir[idx - 1] = temp;
            }
          }
        }
      }
    } else {
      int nullStartIdx;
      int nullStartIdx_tmp;
      nullStartIdx_tmp = qrmanager->ldq * qrmanager->ncols;
      nullStartIdx = nullStartIdx_tmp + 1;
      if (objective->objtype == 5) {
        int A_maxDiag_idx;
        for (idx = 0; idx < mNull_tmp; idx++) {
          memspace->workspace_double[idx] =
              -qrmanager
                   ->Q[nVar_tmp + qrmanager->ldq * (qrmanager->ncols + idx)];
        }
        A_maxDiag_idx = qrmanager->ldq;
        if (qrmanager->mrows != 0) {
          int i;
          int ldw;
          for (int nVars{0}; nVars <= nVar_tmp; nVars++) {
            solution->searchDir[nVars] = 0.0;
          }
          ldw = 0;
          i = (nullStartIdx_tmp + qrmanager->ldq * (mNull_tmp - 1)) + 1;
          for (idx = nullStartIdx; A_maxDiag_idx < 0 ? idx >= i : idx <= i;
               idx += A_maxDiag_idx) {
            int i1;
            i1 = idx + nVar_tmp;
            for (int ldQ{idx}; ldQ <= i1; ldQ++) {
              int i2;
              i2 = ldQ - idx;
              solution->searchDir[i2] =
                  solution->searchDir[i2] +
                  qrmanager->Q[ldQ - 1] * memspace->workspace_double[ldw];
            }
            ldw++;
          }
        }
      } else {
        double temp;
        int A_maxDiag_idx;
        int i;
        int i1;
        int i2;
        int ldQ;
        int ldw;
        int nVars;
        if (objective->objtype == 3) {
          int ar;
          int br;
          int lastColC;
          nVars = qrmanager->mrows;
          ldw = memspace->workspace_double.size(0);
          idx = cholmanager->ldm;
          ldQ = qrmanager->ldq;
          if (qrmanager->mrows != 0) {
            br = nullStartIdx_tmp;
            lastColC = memspace->workspace_double.size(0) * (mNull_tmp - 1);
            for (int cr{0}; ldw < 0 ? cr >= lastColC : cr <= lastColC;
                 cr += ldw) {
              i = cr + 1;
              i1 = cr + nVars;
              for (int ic{i}; ic <= i1; ic++) {
                memspace->workspace_double[ic - 1] = 0.0;
              }
            }
            for (int cr{0}; ldw < 0 ? cr >= lastColC : cr <= lastColC;
                 cr += ldw) {
              ar = -1;
              i = br + 1;
              i1 = br + nVars;
              for (int ib{i}; ib <= i1; ib++) {
                i2 = cr + 1;
                A_maxDiag_idx = cr + nVars;
                for (int ic{i2}; ic <= A_maxDiag_idx; ic++) {
                  memspace->workspace_double[ic - 1] =
                      memspace->workspace_double[ic - 1] +
                      qrmanager->Q[ib - 1] * H[(ar + ic) - cr];
                }
                ar += nVars;
              }
              br += ldQ;
            }
          }
          lastColC = cholmanager->ldm * (mNull_tmp - 1);
          for (int cr{0}; idx < 0 ? cr >= lastColC : cr <= lastColC;
               cr += idx) {
            i = cr + 1;
            i1 = cr + mNull_tmp;
            for (int ic{i}; ic <= i1; ic++) {
              cholmanager->FMat[ic - 1] = 0.0;
            }
          }
          br = -1;
          for (int cr{0}; idx < 0 ? cr >= lastColC : cr <= lastColC;
               cr += idx) {
            ar = nullStartIdx_tmp;
            i = cr + 1;
            i1 = cr + mNull_tmp;
            for (int ic{i}; ic <= i1; ic++) {
              temp = 0.0;
              for (A_maxDiag_idx = 0; A_maxDiag_idx < nVars; A_maxDiag_idx++) {
                temp += qrmanager->Q[A_maxDiag_idx + ar] *
                        memspace->workspace_double[(A_maxDiag_idx + br) + 1];
              }
              cholmanager->FMat[ic - 1] = cholmanager->FMat[ic - 1] + temp;
              ar += ldQ;
            }
            br += ldw;
          }
        }
        temp = 1.4901161193847656E-6 * static_cast<double>(mNull_tmp);
        ldw = cholmanager->ldm + 1;
        cholmanager->ndims = mNull_tmp;
        A_maxDiag_idx = 0;
        if (mNull_tmp > 1) {
          double smax;
          smax = std::abs(cholmanager->FMat[0]);
          for (ldQ = 2; ldQ <= mNull_tmp; ldQ++) {
            double s;
            s = std::abs(cholmanager->FMat[(ldQ - 1) * ldw]);
            if (s > smax) {
              A_maxDiag_idx = ldQ - 1;
              smax = s;
            }
          }
        }
        cholmanager->regTol_ = std::fmax(
            std::abs(cholmanager->FMat[A_maxDiag_idx +
                                       cholmanager->ldm * A_maxDiag_idx]) *
                2.2204460492503131E-16,
            temp);
        if ((cholmanager->FMat.size(0) > 16384) && (mNull_tmp > 128)) {
          bool exitg1;
          ldQ = 0;
          exitg1 = false;
          while ((!exitg1) && (ldQ < mNull_tmp)) {
            nVars = ldw * ldQ + 1;
            A_maxDiag_idx = mNull_tmp - ldQ;
            if (ldQ + 48 <= mNull_tmp) {
              DynamicRegCholManager::partialColLDL3_(cholmanager, nVars,
                                                     A_maxDiag_idx, temp);
              ldQ += 48;
            } else {
              DynamicRegCholManager::fullColLDL2_(cholmanager, nVars,
                                                  A_maxDiag_idx, temp);
              exitg1 = true;
            }
          }
        } else {
          DynamicRegCholManager::fullColLDL2_(cholmanager, 1, mNull_tmp, temp);
        }
        if (cholmanager->ConvexCheck) {
          idx = 0;
          int exitg2;
          do {
            exitg2 = 0;
            if (idx <= mNull_tmp - 1) {
              if (cholmanager->FMat[idx + cholmanager->ldm * idx] <= 0.0) {
                cholmanager->info = -idx - 1;
                exitg2 = 1;
              } else {
                idx++;
              }
            } else {
              cholmanager->ConvexCheck = false;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          A_maxDiag_idx = qrmanager->ldq;
          if (qrmanager->mrows != 0) {
            for (nVars = 0; nVars < mNull_tmp; nVars++) {
              memspace->workspace_double[nVars] = 0.0;
            }
            nVars = 0;
            i = (nullStartIdx_tmp + qrmanager->ldq * (mNull_tmp - 1)) + 1;
            for (idx = nullStartIdx; A_maxDiag_idx < 0 ? idx >= i : idx <= i;
                 idx += A_maxDiag_idx) {
              temp = 0.0;
              i1 = idx + nVar_tmp;
              for (ldQ = idx; ldQ <= i1; ldQ++) {
                temp += qrmanager->Q[ldQ - 1] * objective->grad[ldQ - idx];
              }
              memspace->workspace_double[nVars] =
                  memspace->workspace_double[nVars] + -temp;
              nVars++;
            }
          }
          nVars = cholmanager->ndims - 2;
          if (cholmanager->ndims != 0) {
            for (idx = 0; idx <= nVars + 1; idx++) {
              A_maxDiag_idx = idx + idx * cholmanager->ldm;
              i = nVars - idx;
              for (ldQ = 0; ldQ <= i; ldQ++) {
                ldw = (idx + ldQ) + 1;
                memspace->workspace_double[ldw] =
                    memspace->workspace_double[ldw] -
                    memspace->workspace_double[idx] *
                        cholmanager->FMat[(A_maxDiag_idx + ldQ) + 1];
              }
            }
          }
          i = cholmanager->ndims;
          for (idx = 0; idx < i; idx++) {
            memspace->workspace_double[idx] =
                memspace->workspace_double[idx] /
                cholmanager->FMat[idx + cholmanager->ldm * idx];
          }
          nVars = cholmanager->ndims;
          if (cholmanager->ndims != 0) {
            for (idx = nVars; idx >= 1; idx--) {
              A_maxDiag_idx = (idx - 1) * cholmanager->ldm;
              temp = memspace->workspace_double[idx - 1];
              i = idx + 1;
              for (ldQ = nVars; ldQ >= i; ldQ--) {
                temp -= cholmanager->FMat[(A_maxDiag_idx + ldQ) - 1] *
                        memspace->workspace_double[ldQ - 1];
              }
              memspace->workspace_double[idx - 1] = temp;
            }
          }
          A_maxDiag_idx = qrmanager->ldq;
          if (qrmanager->mrows != 0) {
            for (nVars = 0; nVars <= nVar_tmp; nVars++) {
              solution->searchDir[nVars] = 0.0;
            }
            ldw = 0;
            i = (nullStartIdx_tmp + qrmanager->ldq * (mNull_tmp - 1)) + 1;
            for (idx = nullStartIdx; A_maxDiag_idx < 0 ? idx >= i : idx <= i;
                 idx += A_maxDiag_idx) {
              i1 = idx + nVar_tmp;
              for (ldQ = idx; ldQ <= i1; ldQ++) {
                i2 = ldQ - idx;
                solution->searchDir[i2] =
                    solution->searchDir[i2] +
                    qrmanager->Q[ldQ - 1] * memspace->workspace_double[ldw];
              }
              ldw++;
            }
          }
        }
      }
    }
  }
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for compute_deltax.cpp
//
// [EOF]
//
