//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeFval_ReuseHx.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "computeFval_ReuseHx.h"
#include "rt_nonfinite.h"
#include "solveLSQ_internal_types.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : const b_struct_T *obj
//                ::coder::array<double, 2U> &workspace
//                const ::coder::array<double, 1U> &f
//                const ::coder::array<double, 1U> &x
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
double computeFval_ReuseHx(const b_struct_T *obj,
                           ::coder::array<double, 2U> &workspace,
                           const ::coder::array<double, 1U> &f,
                           const ::coder::array<double, 1U> &x)
{
  double val;
  switch (obj->objtype) {
  case 5:
    val = obj->gammaScalar * x[obj->nvar - 1];
    break;
  case 3: {
    int i;
    i = obj->nvar;
    for (int b_i{0}; b_i < i; b_i++) {
      workspace[b_i] = 0.5 * obj->Hx[b_i] + f[b_i];
    }
    val = 0.0;
    for (int b_i{0}; b_i < i; b_i++) {
      val += x[b_i] * workspace[b_i];
    }
  } break;
  default: {
    int i;
    int maxRegVar_tmp;
    maxRegVar_tmp = obj->maxVar;
    i = obj->nvar;
    for (int b_i{0}; b_i < i; b_i++) {
      workspace[b_i] = f[b_i];
    }
    i = obj->maxVar - obj->nvar;
    for (int b_i{0}; b_i <= i - 2; b_i++) {
      workspace[obj->nvar + b_i] = 0.0;
    }
    for (int b_i{0}; b_i <= maxRegVar_tmp - 2; b_i++) {
      workspace[b_i] = workspace[b_i] + 0.5 * obj->Hx[b_i];
    }
    val = 0.0;
    for (int b_i{0}; b_i <= maxRegVar_tmp - 2; b_i++) {
      val += x[b_i] * workspace[b_i];
    }
  } break;
  }
  return val;
}

} // namespace Objective
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeFval_ReuseHx.cpp
//
// [EOF]
//
