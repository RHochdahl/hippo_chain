//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: removeConstr.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "removeConstr.h"
#include "../rt_nonfinite.h"
#include "solveLSQ_internal_types.h"
#include "../coder_array.h"

// Function Definitions
//
// Arguments    : f_struct_T *obj
//                int idx_global
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void removeConstr(f_struct_T *obj, int idx_global)
{
  int TYPE_tmp;
  int i;
  TYPE_tmp = obj->Wid[idx_global - 1] - 1;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE_tmp] +
                       obj->Wlocalidx[idx_global - 1]) -
                      2] = false;
  obj->Wid[idx_global - 1] = obj->Wid[obj->nActiveConstr - 1];
  obj->Wlocalidx[idx_global - 1] = obj->Wlocalidx[obj->nActiveConstr - 1];
  i = obj->nVar;
  for (int idx{0}; idx < i; idx++) {
    obj->ATwset[idx + obj->ldA * (idx_global - 1)] =
        obj->ATwset[idx + obj->ldA * (obj->nActiveConstr - 1)];
  }
  obj->bwset[idx_global - 1] = obj->bwset[obj->nActiveConstr - 1];
  obj->nActiveConstr--;
  obj->nWConstr[TYPE_tmp]--;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for removeConstr.cpp
//
// [EOF]
//
