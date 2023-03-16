//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: countsort.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "countsort.h"
#include "../rt_nonfinite.h"
#include "../coder_array.h"

// Function Definitions
//
// Arguments    : ::coder::array<int, 1U> &x
//                int xLen
//                ::coder::array<int, 1U> &workspace
//                int xMin
//                int xMax
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace utils {
void countsort(::coder::array<int, 1U> &x, int xLen,
               ::coder::array<int, 1U> &workspace, int xMin, int xMax)
{
  if ((xLen > 1) && (xMax > xMin)) {
    int idxEnd;
    int idxStart;
    int maxOffset;
    idxStart = xMax - xMin;
    for (int idx{0}; idx <= idxStart; idx++) {
      workspace[idx] = 0;
    }
    maxOffset = idxStart - 1;
    for (int idx{0}; idx < xLen; idx++) {
      idxStart = x[idx] - xMin;
      workspace[idxStart] = workspace[idxStart] + 1;
    }
    for (int idx{2}; idx <= maxOffset + 2; idx++) {
      workspace[idx - 1] = workspace[idx - 1] + workspace[idx - 2];
    }
    idxStart = 1;
    idxEnd = workspace[0];
    for (int idx{0}; idx <= maxOffset; idx++) {
      for (int idxFill{idxStart}; idxFill <= idxEnd; idxFill++) {
        x[idxFill - 1] = idx + xMin;
      }
      idxStart = workspace[idx] + 1;
      idxEnd = workspace[idx + 1];
    }
    for (int idx{idxStart}; idx <= idxEnd; idx++) {
      x[idx - 1] = xMax;
    }
  }
}

} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for countsort.cpp
//
// [EOF]
//
