//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: PresolveWorkingSet.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 02-Jan-2023 15:29:42
//

// Include Files
#include "PresolveWorkingSet.h"
#include "RemoveDependentEq_.h"
#include "RemoveDependentIneq_.h"
#include "feasibleX0ForWorkingSet.h"
#include "solve_internal_types.h"
#include "maxConstraintViolation.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : struct_T *solution
//                e_struct_T *memspace
//                f_struct_T *workingset
//                c_struct_T *qrmanager
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
void PresolveWorkingSet(struct_T *solution, e_struct_T *memspace,
                        f_struct_T *workingset, c_struct_T *qrmanager)
{
  int idxStartIneq;
  solution->state = 82;
  idxStartIneq = RemoveDependentEq_(memspace, workingset, qrmanager);
  if ((idxStartIneq != -1) && (workingset->nActiveConstr <= qrmanager->ldq)) {
    boolean_T guard1{false};
    boolean_T okWorkingSet;
    RemoveDependentIneq_(workingset, qrmanager, memspace);
    okWorkingSet = feasibleX0ForWorkingSet(
        memspace->workspace_double, solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      b_RemoveDependentIneq_(workingset, qrmanager, memspace);
      okWorkingSet = feasibleX0ForWorkingSet(
          memspace->workspace_double, solution->xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                   workingset->nVar)) {
      double constrViolation;
      constrViolation =
          WorkingSet::maxConstraintViolation(workingset, solution->xstar);
      if (constrViolation > 1.0E-8) {
        solution->state = -2;
      }
    }
  } else {
    int idxEndIneq;
    solution->state = -3;
    idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
    idxEndIneq = workingset->nActiveConstr;
    for (int idx_global{idxStartIneq}; idx_global <= idxEndIneq; idx_global++) {
      workingset->isActiveConstr
          [(workingset->isActiveIdx[workingset->Wid[idx_global - 1] - 1] +
            workingset->Wlocalidx[idx_global - 1]) -
           2] = false;
    }
    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr =
        workingset->nWConstr[0] + workingset->nWConstr[1];
  }
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for PresolveWorkingSet.cpp
//
// [EOF]
//
