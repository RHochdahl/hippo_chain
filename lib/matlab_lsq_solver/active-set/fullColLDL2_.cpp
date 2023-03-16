//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fullColLDL2_.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "fullColLDL2_.h"
#include "../rt_nonfinite.h"
#include "solveLSQ_internal_types.h"
#include "../coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : d_struct_T *obj
//                int LD_offset
//                int NColsRemain
//                double REG_PRIMAL
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace DynamicRegCholManager {
void fullColLDL2_(d_struct_T *obj, int LD_offset, int NColsRemain,
                  double REG_PRIMAL)
{
  int LDimSizeP1;
  int jA;
  LDimSizeP1 = obj->ldm;
  for (int k{0}; k < NColsRemain; k++) {
    double alpha1;
    int LD_diagOffset;
    int i;
    int subMatrixDim;
    LD_diagOffset = (LD_offset + (LDimSizeP1 + 1) * k) - 1;
    if (std::abs(obj->FMat[LD_diagOffset]) <= obj->regTol_) {
      obj->FMat[LD_diagOffset] = obj->FMat[LD_diagOffset] + REG_PRIMAL;
    }
    alpha1 = -1.0 / obj->FMat[LD_diagOffset];
    subMatrixDim = (NColsRemain - k) - 2;
    for (jA = 0; jA <= subMatrixDim; jA++) {
      obj->workspace_[jA] = obj->FMat[(LD_diagOffset + jA) + 1];
    }
    if (!(alpha1 == 0.0)) {
      jA = LD_diagOffset + LDimSizeP1;
      for (int j{0}; j <= subMatrixDim; j++) {
        if (obj->workspace_[j] != 0.0) {
          double temp;
          int i1;
          temp = obj->workspace_[j] * alpha1;
          i = jA + 2;
          i1 = subMatrixDim + jA;
          for (int ijA{i}; ijA <= i1 + 2; ijA++) {
            obj->FMat[ijA - 1] =
                obj->FMat[ijA - 1] + obj->workspace_[(ijA - jA) - 2] * temp;
          }
        }
        jA += obj->ldm;
      }
    }
    for (jA = 0; jA <= subMatrixDim; jA++) {
      i = (LD_diagOffset + jA) + 1;
      obj->FMat[i] = obj->FMat[i] / obj->FMat[LD_diagOffset];
    }
  }
  jA = (LD_offset + (obj->ldm + 1) * (NColsRemain - 1)) - 1;
  if (std::abs(obj->FMat[jA]) <= obj->regTol_) {
    obj->FMat[jA] = obj->FMat[jA] + REG_PRIMAL;
  }
}

} // namespace DynamicRegCholManager
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for fullColLDL2_.cpp
//
// [EOF]
//
