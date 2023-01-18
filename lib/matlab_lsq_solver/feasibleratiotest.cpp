//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: feasibleratiotest.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "feasibleratiotest.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 1U> &solution_xstar
//                const ::coder::array<double, 1U> &solution_searchDir
//                int workingset_nVar
//                const ::coder::array<double, 1U> &workingset_lb
//                const ::coder::array<int, 1U> &workingset_indexLB
//                const ::coder::array<int, 1U> &workingset_indexUB
//                const int workingset_sizes[5]
//                const int workingset_isActiveIdx[6]
//                const ::coder::array<bool, 1U> &workingset_isActiveConstr
//                const int workingset_nWConstr[5]
//                bool isPhaseOne
//                double *alpha
//                bool *newBlocking
//                int *constrType
//                int *constrIdx
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void feasibleratiotest(
    const ::coder::array<double, 1U> &solution_xstar,
    const ::coder::array<double, 1U> &solution_searchDir, int workingset_nVar,
    const ::coder::array<double, 1U> &workingset_lb,
    const ::coder::array<int, 1U> &workingset_indexLB,
    const ::coder::array<int, 1U> &workingset_indexUB,
    const int workingset_sizes[5], const int workingset_isActiveIdx[6],
    const ::coder::array<bool, 1U> &workingset_isActiveConstr,
    const int workingset_nWConstr[5], bool isPhaseOne, double *alpha,
    bool *newBlocking, int *constrType, int *constrIdx)
{
  double denomTol;
  double phaseOneCorrectionP;
  double phaseOneCorrectionX;
  double pk_corrected;
  double ratio;
  int totalUB;
  totalUB = workingset_sizes[4];
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  denomTol = 2.2204460492503131E-13 *
             internal::blas::xnrm2(workingset_nVar, solution_searchDir);
  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    int i;
    phaseOneCorrectionX =
        static_cast<double>(isPhaseOne) * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = static_cast<double>(isPhaseOne) *
                          solution_searchDir[workingset_nVar - 1];
    i = workingset_sizes[3];
    for (int idx{0}; idx <= i - 2; idx++) {
      pk_corrected = -solution_searchDir[workingset_indexLB[idx] - 1] -
                     phaseOneCorrectionP;
      if ((pk_corrected > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[3] + idx) - 1])) {
        ratio = (-solution_xstar[workingset_indexLB[idx] - 1] -
                 workingset_lb[workingset_indexLB[idx] - 1]) -
                phaseOneCorrectionX;
        pk_corrected =
            std::fmin(std::abs(ratio), 1.0E-8 - ratio) / pk_corrected;
        if (pk_corrected < *alpha) {
          *alpha = pk_corrected;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }
    i = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    pk_corrected = -solution_searchDir[i];
    if ((pk_corrected > denomTol) &&
        (!workingset_isActiveConstr
             [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio = -solution_xstar[i] - workingset_lb[i];
      pk_corrected = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / pk_corrected;
      if (pk_corrected < *alpha) {
        *alpha = pk_corrected;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
    }
  }
  if (workingset_nWConstr[4] < workingset_sizes[4]) {
    phaseOneCorrectionX =
        static_cast<double>(isPhaseOne) * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = static_cast<double>(isPhaseOne) *
                          solution_searchDir[workingset_nVar - 1];
    for (int idx{0}; idx < totalUB; idx++) {
      pk_corrected =
          solution_searchDir[workingset_indexUB[idx] - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[4] + idx) - 1])) {
        ratio = (solution_xstar[workingset_indexUB[idx] - 1] - 1.0) -
                phaseOneCorrectionX;
        pk_corrected =
            std::fmin(std::abs(ratio), 1.0E-8 - ratio) / pk_corrected;
        if (pk_corrected < *alpha) {
          *alpha = pk_corrected;
          *constrType = 5;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }
  }
  if (!isPhaseOne) {
    if ((*newBlocking) && (*alpha > 1.0)) {
      *newBlocking = false;
    }
    *alpha = std::fmin(*alpha, 1.0);
  }
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for feasibleratiotest.cpp
//
// [EOF]
//
