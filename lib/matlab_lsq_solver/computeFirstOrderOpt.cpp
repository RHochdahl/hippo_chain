//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeFirstOrderOpt.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 02-Jan-2023 15:29:42
//

// Include Files
#include "computeFirstOrderOpt.h"
#include "solve_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : struct_T *solution
//                const b_struct_T *objective
//                int workingset_nVar
//                int workingset_ldA
//                const ::coder::array<double, 1U> &workingset_ATwset
//                int workingset_nActiveConstr
//                ::coder::array<double, 2U> &workspace
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace parseoutput {
void computeFirstOrderOpt(struct_T *solution, const b_struct_T *objective,
                          int workingset_nVar, int workingset_ldA,
                          const ::coder::array<double, 1U> &workingset_ATwset,
                          int workingset_nActiveConstr,
                          ::coder::array<double, 2U> &workspace)
{
  int ix;
  int k;
  for (k = 0; k < workingset_nVar; k++) {
    workspace[k] = objective->grad[k];
  }
  if (workingset_nActiveConstr != 0) {
    ix = 0;
    k = workingset_ldA * (workingset_nActiveConstr - 1) + 1;
    for (int iac{1}; workingset_ldA < 0 ? iac >= k : iac <= k;
         iac += workingset_ldA) {
      int i;
      i = (iac + workingset_nVar) - 1;
      for (int ia{iac}; ia <= i; ia++) {
        int i1;
        i1 = ia - iac;
        workspace[i1] =
            workspace[i1] + workingset_ATwset[ia - 1] * solution->lambda[ix];
      }
      ix++;
    }
  }
  if (workingset_nVar < 1) {
    ix = 0;
  } else {
    ix = 1;
    if (workingset_nVar > 1) {
      double smax;
      smax = std::abs(workspace[0]);
      for (k = 2; k <= workingset_nVar; k++) {
        double s;
        s = std::abs(workspace[k - 1]);
        if (s > smax) {
          ix = k;
          smax = s;
        }
      }
    }
  }
  solution->firstorderopt = std::abs(workspace[ix - 1]);
}

} // namespace parseoutput
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeFirstOrderOpt.cpp
//
// [EOF]
//
