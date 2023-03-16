//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeFval.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "computeFval.h"
#include "../rt_nonfinite.h"
#include "solveLSQ_internal_types.h"
#include "../coder_array.h"

// Function Definitions
//
// Arguments    : const b_struct_T *obj
//                ::coder::array<double, 2U> &workspace
//                const ::coder::array<double, 2U> &H
//                const ::coder::array<double, 1U> &f
//                const ::coder::array<double, 1U> &x
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
double computeFval(const b_struct_T *obj, ::coder::array<double, 2U> &workspace,
                   const ::coder::array<double, 2U> &H,
                   const ::coder::array<double, 1U> &f,
                   const ::coder::array<double, 1U> &x)
{
  double val;
  switch (obj->objtype) {
  case 5:
    val = x[obj->nvar - 1];
    break;
  case 3: {
    int i;
    int ix;
    int k;
    int lda;
    lda = obj->nvar;
    for (ix = 0; ix < lda; ix++) {
      workspace[ix] = f[ix];
    }
    ix = 0;
    i = obj->nvar * (obj->nvar - 1) + 1;
    for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      double c;
      int i1;
      c = 0.5 * x[ix];
      i1 = (iac + obj->nvar) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        k = ia - iac;
        workspace[k] = workspace[k] + H[ia - 1] * c;
      }
      ix++;
    }
    val = 0.0;
    for (k = 0; k < lda; k++) {
      val += x[k] * workspace[k];
    }
  } break;
  default: {
    int i;
    int i1;
    int ix;
    int k;
    int lda;
    lda = obj->nvar;
    for (ix = 0; ix < lda; ix++) {
      workspace[ix] = f[ix];
    }
    ix = 0;
    i = obj->nvar * (obj->nvar - 1) + 1;
    for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      double c;
      c = 0.5 * x[ix];
      i1 = (iac + obj->nvar) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        k = ia - iac;
        workspace[k] = workspace[k] + H[ia - 1] * c;
      }
      ix++;
    }
    i = obj->nvar + 1;
    i1 = obj->maxVar - 1;
    for (ix = i; ix <= i1; ix++) {
      workspace[ix - 1] = 0.0 * x[ix - 1];
    }
    val = 0.0;
    ix = obj->maxVar;
    for (k = 0; k <= ix - 2; k++) {
      val += x[k] * workspace[k];
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
// File trailer for computeFval.cpp
//
// [EOF]
//
