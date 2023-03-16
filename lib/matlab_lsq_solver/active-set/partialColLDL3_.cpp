//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: partialColLDL3_.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "partialColLDL3_.h"
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
void partialColLDL3_(d_struct_T *obj, int LD_offset, int NColsRemain,
                     double REG_PRIMAL)
{
  int FMat_offset;
  int LD_diagOffset;
  int LDimSizeP1;
  int i;
  int i1;
  int i2;
  int i3;
  int idx;
  int ix;
  int k;
  int lda;
  int offsetColK;
  int subRows;
  LDimSizeP1 = obj->ldm + 1;
  i = NColsRemain - 1;
  for (k = 0; k < 48; k++) {
    subRows = (NColsRemain - k) - 1;
    FMat_offset = LDimSizeP1 * k;
    LD_diagOffset = (LD_offset + FMat_offset) - 1;
    for (idx = 0; idx <= subRows; idx++) {
      obj->workspace_[FMat_offset + idx] = obj->FMat[LD_diagOffset + idx];
    }
    offsetColK = obj->ldm * k;
    for (idx = 0; idx <= i; idx++) {
      obj->workspace2_[idx] = obj->workspace_[offsetColK + idx];
    }
    lda = obj->ldm;
    if ((NColsRemain != 0) && (k != 0)) {
      ix = LD_offset + k;
      i1 = obj->ldm * (k - 1) + 1;
      for (int iac{1}; lda < 0 ? iac >= i1 : iac <= i1; iac += lda) {
        i2 = (iac + NColsRemain) - 1;
        for (idx = iac; idx <= i2; idx++) {
          i3 = idx - iac;
          obj->workspace2_[i3] = obj->workspace2_[i3] +
                                 obj->workspace_[idx - 1] * -obj->FMat[ix - 1];
        }
        ix += obj->ldm;
      }
    }
    for (idx = 0; idx <= i; idx++) {
      obj->workspace_[offsetColK + idx] = obj->workspace2_[idx];
    }
    for (idx = 0; idx <= subRows; idx++) {
      obj->FMat[LD_diagOffset + idx] = obj->workspace_[FMat_offset + idx];
    }
    if (std::abs(obj->FMat[LD_diagOffset]) <= obj->regTol_) {
      obj->FMat[LD_diagOffset] = obj->FMat[LD_diagOffset] + REG_PRIMAL;
    }
    for (idx = 0; idx < subRows; idx++) {
      i1 = (LD_diagOffset + idx) + 1;
      obj->FMat[i1] = obj->FMat[i1] / obj->FMat[LD_diagOffset];
    }
  }
  for (int j{48}; j <= i; j += 48) {
    int ia0;
    int m;
    int subBlockSize;
    subRows = NColsRemain - j;
    if (subRows >= 48) {
      subBlockSize = 48;
    } else {
      subBlockSize = subRows;
    }
    i1 = (j + subBlockSize) - 1;
    for (k = j; k <= i1; k++) {
      m = (j + subBlockSize) - k;
      LD_diagOffset = (LD_offset + LDimSizeP1 * k) - 1;
      for (idx = 0; idx < 48; idx++) {
        obj->workspace2_[idx] =
            obj->FMat[((LD_offset + k) + idx * obj->ldm) - 1];
      }
      ia0 = k + 1;
      lda = obj->ldm;
      if (m != 0) {
        ix = 0;
        i2 = (k + obj->ldm * 47) + 1;
        for (int iac{ia0}; lda < 0 ? iac >= i2 : iac <= i2; iac += lda) {
          i3 = (iac + m) - 1;
          for (idx = iac; idx <= i3; idx++) {
            FMat_offset = (LD_diagOffset + idx) - iac;
            obj->FMat[FMat_offset] =
                obj->FMat[FMat_offset] +
                obj->workspace_[idx - 1] * -obj->workspace2_[ix];
          }
          ix++;
        }
      }
    }
    if (j + subBlockSize < NColsRemain) {
      m = subRows - subBlockSize;
      ia0 = j + subBlockSize;
      k = ((LD_offset + subBlockSize) + LDimSizeP1 * j) - 1;
      i1 = subBlockSize - 1;
      for (idx = 0; idx < 48; idx++) {
        subRows = idx * obj->ldm;
        FMat_offset = (LD_offset + j) + subRows;
        for (LD_diagOffset = 0; LD_diagOffset <= i1; LD_diagOffset++) {
          obj->workspace2_[subRows + LD_diagOffset] =
              obj->FMat[(FMat_offset + LD_diagOffset) - 1];
        }
      }
      offsetColK = obj->ldm;
      idx = obj->ldm;
      if ((m != 0) && (subBlockSize != 0)) {
        FMat_offset = k + obj->ldm * (subBlockSize - 1);
        subRows = 0;
        for (lda = k; idx < 0 ? lda >= FMat_offset : lda <= FMat_offset;
             lda += idx) {
          LD_diagOffset = ia0 - 1;
          subRows++;
          i1 = subRows + offsetColK * 47;
          for (ix = subRows; offsetColK < 0 ? ix >= i1 : ix <= i1;
               ix += offsetColK) {
            i2 = lda + 1;
            i3 = lda + m;
            for (int iac{i2}; iac <= i3; iac++) {
              obj->FMat[iac - 1] =
                  obj->FMat[iac - 1] +
                  -obj->workspace2_[ix - 1] *
                      obj->workspace_[(LD_diagOffset + iac) - lda];
            }
            LD_diagOffset += obj->ldm;
          }
        }
      }
    }
  }
}

} // namespace DynamicRegCholManager
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for partialColLDL3_.cpp
//
// [EOF]
//
