//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ratiotest.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "ratiotest.h"
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
//                double *toldelta
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
void ratiotest(const ::coder::array<double, 1U> &solution_xstar,
               const ::coder::array<double, 1U> &solution_searchDir,
               int workingset_nVar,
               const ::coder::array<double, 1U> &workingset_lb,
               const ::coder::array<int, 1U> &workingset_indexLB,
               const ::coder::array<int, 1U> &workingset_indexUB,
               const int workingset_sizes[5],
               const int workingset_isActiveIdx[6],
               const ::coder::array<bool, 1U> &workingset_isActiveConstr,
               const int workingset_nWConstr[5], double *toldelta,
               double *alpha, bool *newBlocking, int *constrType,
               int *constrIdx)
{
  double denomTol;
  double p_max;
  double phaseOneCorrectionP;
  double phaseOneCorrectionX;
  double pk_corrected;
  double ratio;
  double ratio_tmp;
  int totalUB;
  totalUB = workingset_sizes[4];
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  p_max = 0.0;
  denomTol = 2.2204460492503131E-13 *
             internal::blas::xnrm2(workingset_nVar, solution_searchDir);
  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    int i;
    phaseOneCorrectionX = 0.0 * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = 0.0 * solution_searchDir[workingset_nVar - 1];
    i = workingset_sizes[3];
    for (int idx{0}; idx <= i - 2; idx++) {
      pk_corrected = -solution_searchDir[workingset_indexLB[idx] - 1] -
                     phaseOneCorrectionP;
      if ((pk_corrected > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[3] + idx) - 1])) {
        ratio_tmp = -solution_xstar[workingset_indexLB[idx] - 1] -
                    workingset_lb[workingset_indexLB[idx] - 1];
        ratio = (ratio_tmp - *toldelta) - phaseOneCorrectionX;
        ratio = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / pk_corrected;
        if ((ratio <= *alpha) && (std::abs(pk_corrected) > p_max)) {
          *alpha = ratio;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
        ratio = ratio_tmp - phaseOneCorrectionX;
        ratio = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / pk_corrected;
        if (ratio < *alpha) {
          *alpha = ratio;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
          p_max = std::abs(pk_corrected);
        }
      }
    }
    i = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    phaseOneCorrectionX = solution_searchDir[i];
    if ((-phaseOneCorrectionX > denomTol) &&
        (!workingset_isActiveConstr
             [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio_tmp = -solution_xstar[i] - workingset_lb[i];
      ratio = ratio_tmp - *toldelta;
      ratio = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / -phaseOneCorrectionX;
      if ((ratio <= *alpha) && (std::abs(phaseOneCorrectionX) > p_max)) {
        *alpha = ratio;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
      ratio = std::fmin(std::abs(ratio_tmp), 1.0E-8 - ratio_tmp) /
              -phaseOneCorrectionX;
      if (ratio < *alpha) {
        *alpha = ratio;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
        p_max = std::abs(solution_searchDir[i]);
      }
    }
  }
  if (workingset_nWConstr[4] < workingset_sizes[4]) {
    phaseOneCorrectionX = 0.0 * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = 0.0 * solution_searchDir[workingset_nVar - 1];
    for (int idx{0}; idx < totalUB; idx++) {
      pk_corrected =
          solution_searchDir[workingset_indexUB[idx] - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[4] + idx) - 1])) {
        ratio_tmp = solution_xstar[workingset_indexUB[idx] - 1] - 1.0;
        ratio = (ratio_tmp - *toldelta) - phaseOneCorrectionX;
        ratio = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / pk_corrected;
        if ((ratio <= *alpha) && (std::abs(pk_corrected) > p_max)) {
          *alpha = ratio;
          *constrType = 5;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
        ratio = ratio_tmp - phaseOneCorrectionX;
        ratio = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / pk_corrected;
        if (ratio < *alpha) {
          *alpha = ratio;
          *constrType = 5;
          *constrIdx = idx + 1;
          *newBlocking = true;
          p_max = std::abs(pk_corrected);
        }
      }
    }
  }
  *toldelta += 6.608625846508183E-7;
  if (p_max > 0.0) {
    *alpha = std::fmax(*alpha, 6.608625846508183E-7 / p_max);
  }
  if ((*newBlocking) && (*alpha > 1.0)) {
    *newBlocking = false;
  }
  *alpha = std::fmin(*alpha, 1.0);
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for ratiotest.cpp
//
// [EOF]
//
