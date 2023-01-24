//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: setProblemType.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "setProblemType.h"
#include "rt_nonfinite.h"
#include "solveLSQ_internal_types.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : f_struct_T *obj
//                int PROBLEM_TYPE
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void setProblemType(f_struct_T *obj, int PROBLEM_TYPE)
{
  switch (PROBLEM_TYPE) {
  case 3: {
    int i;
    obj->nVar = obj->nVarOrig;
    obj->mConstr = obj->mConstrOrig;
    if (obj->nWConstr[4] > 0) {
      i = obj->sizesNormal[4];
      for (int colOffsetATw{0}; colOffsetATw < i; colOffsetATw++) {
        obj->isActiveConstr[(obj->isActiveIdxNormal[4] + colOffsetATw) - 1] =
            obj->isActiveConstr[(obj->isActiveIdx[4] + colOffsetATw) - 1];
      }
    }
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesNormal[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxNormal[i];
    }
  } break;
  case 1: {
    int i;
    int idxStartIneq;
    obj->nVar = obj->nVarOrig + 1;
    obj->mConstr = obj->mConstrOrig + 1;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxPhaseOne[i];
    }
    i = obj->sizes[0];
    for (int colOffsetATw{0}; colOffsetATw < i; colOffsetATw++) {
      obj->ATwset[(obj->nVar + obj->ldA * colOffsetATw) - 1] = 0.0;
    }
    obj->indexLB[obj->sizes[3] - 1] = obj->nVar;
    obj->lb[obj->nVar - 1] = obj->SLACK0;
    idxStartIneq = obj->isActiveIdx[2];
    i = obj->nActiveConstr;
    for (int colOffsetATw{idxStartIneq}; colOffsetATw <= i; colOffsetATw++) {
      obj->ATwset[(obj->nVar + obj->ldA * (colOffsetATw - 1)) - 1] = -1.0;
    }
    if (obj->nWConstr[4] > 0) {
      i = obj->sizesNormal[4];
      for (int colOffsetATw{0}; colOffsetATw <= i; colOffsetATw++) {
        obj->isActiveConstr[(obj->isActiveIdx[4] + colOffsetATw) - 1] = false;
      }
    }
    obj->isActiveConstr[obj->isActiveIdx[4] - 2] = false;
  } break;
  case 2: {
    int i;
    obj->nVar = obj->nVarMax - 1;
    obj->mConstr = obj->mConstrMax - 1;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegularized[i];
    }
    if (obj->probType != 4) {
      int colOffsetATw;
      int i1;
      int idxStartIneq;
      int idx_lb;
      int offsetIneq_tmp_tmp;
      offsetIneq_tmp_tmp = obj->nVarOrig + 1;
      i = obj->sizes[0];
      for (int idx_col{0}; idx_col < i; idx_col++) {
        colOffsetATw = obj->ldA * idx_col;
        i1 = obj->nVar;
        for (int idx_row{offsetIneq_tmp_tmp}; idx_row <= i1; idx_row++) {
          obj->ATwset[(idx_row + colOffsetATw) - 1] = 0.0;
        }
      }
      idx_lb = obj->nVarOrig;
      i = obj->sizesNormal[3] + 1;
      i1 = obj->sizesRegularized[3];
      for (colOffsetATw = i; colOffsetATw <= i1; colOffsetATw++) {
        idx_lb++;
        obj->indexLB[colOffsetATw - 1] = idx_lb;
      }
      if (obj->nWConstr[4] > 0) {
        i = obj->sizesRegularized[4];
        for (colOffsetATw = 0; colOffsetATw < i; colOffsetATw++) {
          obj->isActiveConstr[obj->isActiveIdxRegularized[4] + colOffsetATw] =
              obj->isActiveConstr[(obj->isActiveIdx[4] + colOffsetATw) - 1];
        }
      }
      i = obj->isActiveIdx[4];
      i1 = obj->isActiveIdxRegularized[4] - 1;
      for (colOffsetATw = i; colOffsetATw <= i1; colOffsetATw++) {
        obj->isActiveConstr[colOffsetATw - 1] = false;
      }
      i = obj->nVarOrig;
      for (colOffsetATw = offsetIneq_tmp_tmp; colOffsetATw <= i;
           colOffsetATw++) {
        obj->lb[colOffsetATw - 1] = 0.0;
      }
      idxStartIneq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (int idx_col{idxStartIneq}; idx_col <= i; idx_col++) {
        colOffsetATw = obj->ldA * (idx_col - 1) - 1;
        if (obj->Wid[idx_col - 1] == 3) {
          idx_lb = obj->Wlocalidx[idx_col - 1];
          i1 = (offsetIneq_tmp_tmp + idx_lb) - 2;
          for (int idx_row{offsetIneq_tmp_tmp}; idx_row <= i1; idx_row++) {
            obj->ATwset[idx_row + colOffsetATw] = 0.0;
          }
          obj->ATwset[((offsetIneq_tmp_tmp + idx_lb) + colOffsetATw) - 1] =
              -1.0;
          i1 = offsetIneq_tmp_tmp + idx_lb;
          idx_lb = obj->nVar;
          for (int idx_row{i1}; idx_row <= idx_lb; idx_row++) {
            obj->ATwset[idx_row + colOffsetATw] = 0.0;
          }
        } else {
          i1 = obj->nVar;
          for (int idx_row{offsetIneq_tmp_tmp}; idx_row <= i1; idx_row++) {
            obj->ATwset[idx_row + colOffsetATw] = 0.0;
          }
        }
      }
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegularized[i];
    }
  } break;
  default: {
    int i;
    int idxStartIneq;
    obj->nVar = obj->nVarMax;
    obj->mConstr = obj->mConstrMax;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegPhaseOne[i];
    }
    i = obj->sizes[0];
    for (int colOffsetATw{0}; colOffsetATw < i; colOffsetATw++) {
      obj->ATwset[(obj->nVar + obj->ldA * colOffsetATw) - 1] = 0.0;
    }
    obj->indexLB[obj->sizes[3] - 1] = obj->nVar;
    obj->lb[obj->nVar - 1] = obj->SLACK0;
    idxStartIneq = obj->isActiveIdx[2];
    i = obj->nActiveConstr;
    for (int colOffsetATw{idxStartIneq}; colOffsetATw <= i; colOffsetATw++) {
      obj->ATwset[(obj->nVar + obj->ldA * (colOffsetATw - 1)) - 1] = -1.0;
    }
    if (obj->nWConstr[4] > 0) {
      i = obj->sizesNormal[4];
      for (int colOffsetATw{0}; colOffsetATw <= i; colOffsetATw++) {
        obj->isActiveConstr[(obj->isActiveIdx[4] + colOffsetATw) - 1] = false;
      }
    }
    obj->isActiveConstr[obj->isActiveIdx[4] - 2] = false;
  } break;
  }
  obj->probType = PROBLEM_TYPE;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for setProblemType.cpp
//
// [EOF]
//
