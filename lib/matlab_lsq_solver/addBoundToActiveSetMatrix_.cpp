//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: addBoundToActiveSetMatrix_.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "addBoundToActiveSetMatrix_.h"
#include "rt_nonfinite.h"
#include "solveLSQ_internal_types.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : f_struct_T *obj
//                int TYPE
//                int idx_local
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void addBoundToActiveSetMatrix_(f_struct_T *obj, int TYPE, int idx_local)
{
  int colOffset;
  int i;
  int idx_bnd_local;
  obj->nWConstr[TYPE - 1]++;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  obj->Wid[obj->nActiveConstr - 1] = TYPE;
  obj->Wlocalidx[obj->nActiveConstr - 1] = idx_local;
  colOffset = obj->ldA * (obj->nActiveConstr - 1) - 1;
  if (TYPE == 5) {
    idx_bnd_local = obj->indexUB[idx_local - 1];
    obj->bwset[obj->nActiveConstr - 1] = 1.0;
  } else {
    idx_bnd_local = obj->indexLB[idx_local - 1];
    obj->bwset[obj->nActiveConstr - 1] = obj->lb[idx_bnd_local - 1];
  }
  for (int idx{0}; idx <= idx_bnd_local - 2; idx++) {
    obj->ATwset[(idx + colOffset) + 1] = 0.0;
  }
  obj->ATwset[idx_bnd_local + colOffset] =
      2.0 * static_cast<double>(TYPE == 5) - 1.0;
  idx_bnd_local++;
  i = obj->nVar;
  for (int idx{idx_bnd_local}; idx <= i; idx++) {
    obj->ATwset[idx + colOffset] = 0.0;
  }
  switch (obj->probType) {
  case 3:
  case 2:
    break;
  default:
    obj->ATwset[obj->nVar + colOffset] = -1.0;
    break;
  }
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for addBoundToActiveSetMatrix_.cpp
//
// [EOF]
//
