//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: iterate.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "iterate.h"
#include "addBoundToActiveSetMatrix_.h"
#include "computeFval_ReuseHx.h"
#include "computeGrad_StoreHx.h"
#include "computeQ_.h"
#include "compute_deltax.h"
#include "deleteColMoveEnd.h"
#include "factorQR.h"
#include "feasibleX0ForWorkingSet.h"
#include "feasibleratiotest.h"
#include "maxConstraintViolation.h"
#include "ratiotest.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "solveLSQ_internal_types.h"
#include "xnrm2.h"
#include "xrotg.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &H
//                const ::coder::array<double, 1U> &f
//                struct_T *solution
//                e_struct_T *memspace
//                f_struct_T *workingset
//                c_struct_T *qrmanager
//                d_struct_T *cholmanager
//                b_struct_T *objective
//                double options_ObjectiveLimit
//                double options_StepTolerance
//                int runTimeOptions_MaxIterations
//                double runTimeOptions_ProbRelTolFactor
//                bool runTimeOptions_RemainFeasible
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void iterate(const ::coder::array<double, 2U> &H,
             const ::coder::array<double, 1U> &f, struct_T *solution,
             e_struct_T *memspace, f_struct_T *workingset,
             c_struct_T *qrmanager, d_struct_T *cholmanager,
             b_struct_T *objective, double options_ObjectiveLimit,
             double options_StepTolerance, int runTimeOptions_MaxIterations,
             double runTimeOptions_ProbRelTolFactor,
             bool runTimeOptions_RemainFeasible)
{
  double c;
  double d;
  double s;
  double temp;
  double tolDelta;
  int TYPE;
  int activeSetChangeID;
  int globalActiveConstrIdx;
  int idxRotGCol;
  int iyend;
  int k;
  int nVar;
  bool newBlocking;
  bool subProblemChanged;
  bool updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  tolDelta = 6.7434957617430445E-7;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  Objective::computeGrad_StoreHx(objective, H, f, solution->xstar);
  solution->fstar = Objective::computeFval_ReuseHx(
      objective, memspace->workspace_double, f, solution->xstar);
  if (solution->iterations < runTimeOptions_MaxIterations) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }
  iyend = workingset->mConstrMax;
  for (k = 0; k < iyend; k++) {
    solution->lambda[k] = 0.0;
  }
  int exitg1;
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      int b_iy;
      int i;
      int idx;
      int iy;
      int nActiveConstr;
      bool guard1{false};
      bool guard2{false};
      guard1 = false;
      guard2 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
        case 1:
          nActiveConstr = workingset->ldA * (workingset->nActiveConstr - 1);
          idxRotGCol = qrmanager->mrows;
          iyend = qrmanager->ncols + 1;
          if (idxRotGCol <= iyend) {
            iyend = idxRotGCol;
          }
          qrmanager->minRowCol = iyend;
          iy = qrmanager->ldq * qrmanager->ncols;
          idxRotGCol = qrmanager->ldq;
          if (qrmanager->mrows != 0) {
            iyend = iy + qrmanager->mrows;
            for (b_iy = iy + 1; b_iy <= iyend; b_iy++) {
              qrmanager->QR[b_iy - 1] = 0.0;
            }
            i = qrmanager->ldq * (qrmanager->mrows - 1) + 1;
            for (iyend = 1; idxRotGCol < 0 ? iyend >= i : iyend <= i;
                 iyend += idxRotGCol) {
              c = 0.0;
              k = (iyend + qrmanager->mrows) - 1;
              for (idx = iyend; idx <= k; idx++) {
                c += qrmanager->Q[idx - 1] *
                     workingset->ATwset[(nActiveConstr + idx) - iyend];
              }
              qrmanager->QR[iy] = qrmanager->QR[iy] + c;
              iy++;
            }
          }
          qrmanager->ncols++;
          qrmanager->jpvt[qrmanager->ncols - 1] = qrmanager->ncols;
          for (idx = qrmanager->mrows - 2; idx + 2 > qrmanager->ncols; idx--) {
            idxRotGCol = qrmanager->ldq * (qrmanager->ncols - 1);
            i = (idx + idxRotGCol) + 1;
            d = qrmanager->QR[i];
            internal::blas::xrotg(&qrmanager->QR[idx + idxRotGCol], &d, &c, &s);
            qrmanager->QR[i] = d;
            idxRotGCol = qrmanager->ldq * idx;
            iyend = qrmanager->mrows;
            if (qrmanager->mrows >= 1) {
              b_iy = qrmanager->ldq + idxRotGCol;
              for (k = 0; k < iyend; k++) {
                iy = b_iy + k;
                nActiveConstr = idxRotGCol + k;
                temp = c * qrmanager->Q[nActiveConstr] + s * qrmanager->Q[iy];
                qrmanager->Q[iy] =
                    c * qrmanager->Q[iy] - s * qrmanager->Q[nActiveConstr];
                qrmanager->Q[nActiveConstr] = temp;
              }
            }
          }
          break;
        case -1:
          QRManager::deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;
        default:
          QRManager::factorQR(qrmanager, workingset->ATwset, nVar,
                              workingset->nActiveConstr, workingset->ldA);
          QRManager::computeQ_(qrmanager, qrmanager->mrows);
          break;
        }
        compute_deltax(H, solution, memspace, qrmanager, cholmanager,
                       objective);
        if (solution->state != -5) {
          exitg1 = 1;
        } else if ((internal::blas::xnrm2(nVar, solution->searchDir) <
                    options_StepTolerance) ||
                   (workingset->nActiveConstr >= nVar)) {
          guard2 = true;
        } else {
          updateFval = (TYPE == 5);
          if (updateFval || runTimeOptions_RemainFeasible) {
            feasibleratiotest(
                solution->xstar, solution->searchDir, workingset->nVar,
                workingset->lb, workingset->indexLB, workingset->indexUB,
                workingset->sizes, workingset->isActiveIdx,
                workingset->isActiveConstr, workingset->nWConstr, updateFval,
                &temp, &newBlocking, &idxRotGCol, &iyend);
          } else {
            ratiotest(solution->xstar, solution->searchDir, workingset->nVar,
                      workingset->lb, workingset->indexLB, workingset->indexUB,
                      workingset->sizes, workingset->isActiveIdx,
                      workingset->isActiveConstr, workingset->nWConstr,
                      &tolDelta, &temp, &newBlocking, &idxRotGCol, &iyend);
          }
          if (newBlocking) {
            switch (idxRotGCol) {
            case 3:
              workingset->nWConstr[2]++;
              workingset
                  ->isActiveConstr[(workingset->isActiveIdx[2] + iyend) - 2] =
                  true;
              workingset->nActiveConstr++;
              workingset->Wid[workingset->nActiveConstr - 1] = 3;
              workingset->Wlocalidx[workingset->nActiveConstr - 1] = iyend;
              idxRotGCol = workingset->ldA * (workingset->nActiveConstr - 1);
              i = workingset->nVar - 1;
              for (idx = 0; idx <= i; idx++) {
                workingset->ATwset[idxRotGCol + idx] = -1.0;
              }
              // A check that is always false is detected at compile-time.
              // Eliminating code that follows.
              break;
            case 4:
              WorkingSet::addBoundToActiveSetMatrix_(workingset, 4, iyend);
              break;
            default:
              WorkingSet::addBoundToActiveSetMatrix_(workingset, 5, iyend);
              break;
            }
            activeSetChangeID = 1;
          } else {
            if (objective->objtype == 5) {
              if (internal::blas::xnrm2(objective->nvar, solution->searchDir) >
                  100.0 * static_cast<double>(objective->nvar) *
                      1.4901161193847656E-8) {
                solution->state = 3;
              } else {
                solution->state = 4;
              }
            }
            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }
          if (!(temp == 0.0)) {
            idxRotGCol = nVar - 1;
            for (k = 0; k <= idxRotGCol; k++) {
              solution->xstar[k] =
                  solution->xstar[k] + temp * solution->searchDir[k];
            }
          }
          Objective::computeGrad_StoreHx(objective, H, f, solution->xstar);
          updateFval = true;
          guard1 = true;
        }
      } else {
        for (k = 0; k < nVar; k++) {
          solution->searchDir[k] = 0.0;
        }
        guard2 = true;
      }
      if (guard2) {
        nActiveConstr = qrmanager->ncols;
        if (qrmanager->ncols > 0) {
          bool b_guard1{false};
          b_guard1 = false;
          if (objective->objtype != 4) {
            temp = 100.0 * static_cast<double>(qrmanager->mrows) *
                   2.2204460492503131E-16;
            if ((qrmanager->mrows > 0) && (qrmanager->ncols > 0)) {
              updateFval = true;
            } else {
              updateFval = false;
            }
            if (updateFval) {
              bool b_guard2{false};
              idx = qrmanager->ncols;
              b_guard2 = false;
              if (qrmanager->mrows < qrmanager->ncols) {
                iyend =
                    qrmanager->mrows + qrmanager->ldq * (qrmanager->ncols - 1);
                while ((idx > qrmanager->mrows) &&
                       (std::abs(qrmanager->QR[iyend - 1]) >= temp)) {
                  idx--;
                  iyend -= qrmanager->ldq;
                }
                updateFval = (idx == qrmanager->mrows);
                if (updateFval) {
                  b_guard2 = true;
                }
              } else {
                b_guard2 = true;
              }
              if (b_guard2) {
                iyend = idx + qrmanager->ldq * (idx - 1);
                while ((idx >= 1) &&
                       (std::abs(qrmanager->QR[iyend - 1]) >= temp)) {
                  idx--;
                  iyend = (iyend - qrmanager->ldq) - 1;
                }
                updateFval = (idx == 0);
              }
            }
            if (!updateFval) {
              solution->state = -7;
            } else {
              b_guard1 = true;
            }
          } else {
            b_guard1 = true;
          }
          if (b_guard1) {
            iy = qrmanager->ldq;
            if (qrmanager->mrows != 0) {
              iyend = qrmanager->ncols;
              for (b_iy = 0; b_iy < iyend; b_iy++) {
                memspace->workspace_double[b_iy] = 0.0;
              }
              b_iy = 0;
              i = qrmanager->ldq * (qrmanager->ncols - 1) + 1;
              for (iyend = 1; iy < 0 ? iyend >= i : iyend <= i; iyend += iy) {
                c = 0.0;
                k = (iyend + qrmanager->mrows) - 1;
                for (idx = iyend; idx <= k; idx++) {
                  c += qrmanager->Q[idx - 1] * objective->grad[idx - iyend];
                }
                memspace->workspace_double[b_iy] =
                    memspace->workspace_double[b_iy] + c;
                b_iy++;
              }
            }
            for (k = nActiveConstr; k >= 1; k--) {
              iyend = (k + (k - 1) * iy) - 1;
              memspace->workspace_double[k - 1] =
                  memspace->workspace_double[k - 1] / qrmanager->QR[iyend];
              for (i = 0; i <= k - 2; i++) {
                idxRotGCol = (k - i) - 2;
                memspace->workspace_double[idxRotGCol] =
                    memspace->workspace_double[idxRotGCol] -
                    memspace->workspace_double[k - 1] *
                        qrmanager->QR[(iyend - i) - 1];
              }
            }
            for (idx = 0; idx < nActiveConstr; idx++) {
              solution->lambda[idx] = -memspace->workspace_double[idx];
            }
          }
        }
        if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
          iyend = 0;
          temp = 0.0 * runTimeOptions_ProbRelTolFactor *
                 static_cast<double>(TYPE != 5);
          i = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          k = workingset->nActiveConstr;
          for (idx = i; idx <= k; idx++) {
            d = solution->lambda[idx - 1];
            if (d < temp) {
              temp = d;
              iyend = idx;
            }
          }
          if (iyend == 0) {
            solution->state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = iyend;
            subProblemChanged = true;
            WorkingSet::removeConstr(workingset, iyend);
            solution->lambda[iyend - 1] = 0.0;
          }
        } else {
          iyend = workingset->nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset->nActiveConstr;
          subProblemChanged = true;
          WorkingSet::removeConstr(workingset, workingset->nActiveConstr);
          solution->lambda[iyend - 1] = 0.0;
        }
        updateFval = false;
        guard1 = true;
      }
      if (guard1) {
        solution->iterations++;
        iyend = objective->nvar - 1;
        if ((solution->iterations >= runTimeOptions_MaxIterations) &&
            ((solution->state != 1) || (objective->objtype == 5))) {
          solution->state = 0;
        }
        if (solution->iterations - solution->iterations / 50 * 50 == 0) {
          solution->maxConstr =
              WorkingSet::maxConstraintViolation(workingset, solution->xstar);
          temp = solution->maxConstr;
          if (objective->objtype == 5) {
            temp = solution->maxConstr - solution->xstar[iyend];
          }
          if (temp > 1.0E-8) {
            for (k = 0; k <= iyend; k++) {
              solution->searchDir[k] = solution->xstar[k];
            }
            newBlocking = initialize::feasibleX0ForWorkingSet(
                memspace->workspace_double, solution->searchDir, workingset,
                qrmanager);
            if ((!newBlocking) && (solution->state != 0)) {
              solution->state = -2;
            }
            activeSetChangeID = 0;
            temp = WorkingSet::maxConstraintViolation(workingset,
                                                      solution->searchDir);
            if (temp < solution->maxConstr) {
              for (idx = 0; idx <= iyend; idx++) {
                solution->xstar[idx] = solution->searchDir[idx];
              }
              solution->maxConstr = temp;
            }
          }
        }
        if (updateFval && (options_ObjectiveLimit > rtMinusInf)) {
          solution->fstar = Objective::computeFval_ReuseHx(
              objective, memspace->workspace_double, f, solution->xstar);
          if ((solution->fstar < options_ObjectiveLimit) &&
              ((solution->state != 0) || (objective->objtype != 5))) {
            solution->state = 2;
          }
        }
      }
    } else {
      if (!updateFval) {
        solution->fstar = Objective::computeFval_ReuseHx(
            objective, memspace->workspace_double, f, solution->xstar);
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for iterate.cpp
//
// [EOF]
//
