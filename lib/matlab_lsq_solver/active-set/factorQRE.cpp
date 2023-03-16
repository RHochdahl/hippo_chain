//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: factorQRE.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "factorQRE.h"
#include "../rt_nonfinite.h"
#include "solveLSQ_internal_types.h"
#include "xnrm2.h"
#include "xzgeqp3.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include "../coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : c_struct_T *obj
//                int mrows
//                int ncols
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace QRManager {
void factorQRE(c_struct_T *obj, int mrows, int ncols)
{
  array<double, 1U> vn1;
  array<double, 1U> vn2;
  array<double, 1U> work;
  double temp;
  if (mrows * ncols == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    int i;
    int i1;
    int ij;
    int ma;
    int minmana;
    obj->usedPivoting = true;
    obj->mrows = mrows;
    obj->ncols = ncols;
    if (mrows <= ncols) {
      i = mrows;
    } else {
      i = ncols;
    }
    obj->minRowCol = i;
    ma = obj->QR.size(0);
    ij = obj->QR.size(0);
    minmana = obj->QR.size(1);
    if (ij <= minmana) {
      minmana = ij;
    }
    obj->tau.set_size(minmana);
    for (i1 = 0; i1 < minmana; i1++) {
      obj->tau[i1] = 0.0;
    }
    if (i < 1) {
      for (int ii{0}; ii < ncols; ii++) {
        obj->jpvt[ii] = ii + 1;
      }
    } else {
      int ii;
      int ix;
      int nfxd;
      int temp_tmp;
      nfxd = 0;
      for (ii = 0; ii < ncols; ii++) {
        if (obj->jpvt[ii] != 0) {
          nfxd++;
          if (ii + 1 != nfxd) {
            ix = ii * ma;
            minmana = (nfxd - 1) * ma;
            for (int k{0}; k < mrows; k++) {
              temp_tmp = ix + k;
              temp = obj->QR[temp_tmp];
              i1 = minmana + k;
              obj->QR[temp_tmp] = obj->QR[i1];
              obj->QR[i1] = temp;
            }
            obj->jpvt[ii] = obj->jpvt[nfxd - 1];
            obj->jpvt[nfxd - 1] = ii + 1;
          } else {
            obj->jpvt[ii] = ii + 1;
          }
        } else {
          obj->jpvt[ii] = ii + 1;
        }
      }
      if (nfxd > i) {
        nfxd = i;
      }
      internal::reflapack::qrf(obj->QR, mrows, ncols, nfxd, obj->tau);
      if (nfxd < i) {
        ma = obj->QR.size(0);
        work.set_size(obj->QR.size(1));
        ij = obj->QR.size(1);
        for (i1 = 0; i1 < ij; i1++) {
          work[i1] = 0.0;
        }
        vn1.set_size(obj->QR.size(1));
        ij = obj->QR.size(1);
        for (i1 = 0; i1 < ij; i1++) {
          vn1[i1] = 0.0;
        }
        vn2.set_size(obj->QR.size(1));
        ij = obj->QR.size(1);
        for (i1 = 0; i1 < ij; i1++) {
          vn2[i1] = 0.0;
        }
        i1 = nfxd + 1;
        for (ii = i1; ii <= ncols; ii++) {
          temp = internal::blas::xnrm2(mrows - nfxd, obj->QR,
                                       (nfxd + (ii - 1) * ma) + 1);
          vn1[ii - 1] = temp;
          vn2[ii - 1] = temp;
        }
        for (int b_i{i1}; b_i <= i; b_i++) {
          double s;
          int ip1;
          int mmi;
          int nmi;
          ip1 = b_i + 1;
          ij = (b_i - 1) * ma;
          ii = (ij + b_i) - 1;
          nmi = (ncols - b_i) + 1;
          mmi = mrows - b_i;
          if (nmi < 1) {
            minmana = -2;
          } else {
            minmana = -1;
            if (nmi > 1) {
              temp = std::abs(vn1[b_i - 1]);
              for (int k{2}; k <= nmi; k++) {
                s = std::abs(vn1[(b_i + k) - 2]);
                if (s > temp) {
                  minmana = k - 2;
                  temp = s;
                }
              }
            }
          }
          nfxd = b_i + minmana;
          if (nfxd + 1 != b_i) {
            ix = nfxd * ma;
            for (int k{0}; k < mrows; k++) {
              temp_tmp = ix + k;
              temp = obj->QR[temp_tmp];
              minmana = ij + k;
              obj->QR[temp_tmp] = obj->QR[minmana];
              obj->QR[minmana] = temp;
            }
            minmana = obj->jpvt[nfxd];
            obj->jpvt[nfxd] = obj->jpvt[b_i - 1];
            obj->jpvt[b_i - 1] = minmana;
            vn1[nfxd] = vn1[b_i - 1];
            vn2[nfxd] = vn2[b_i - 1];
          }
          if (b_i < mrows) {
            temp = obj->QR[ii];
            obj->tau[b_i - 1] =
                internal::reflapack::xzlarfg(mmi + 1, &temp, obj->QR, ii + 2);
            obj->QR[ii] = temp;
          } else {
            obj->tau[b_i - 1] = 0.0;
          }
          if (b_i < ncols) {
            temp = obj->QR[ii];
            obj->QR[ii] = 1.0;
            internal::reflapack::xzlarf(mmi + 1, nmi - 1, ii + 1,
                                        obj->tau[b_i - 1], obj->QR,
                                        (ii + ma) + 1, ma, work);
            obj->QR[ii] = temp;
          }
          for (ii = ip1; ii <= ncols; ii++) {
            ij = b_i + (ii - 1) * ma;
            temp = vn1[ii - 1];
            if (temp != 0.0) {
              double temp2;
              s = std::abs(obj->QR[ij - 1]) / temp;
              s = 1.0 - s * s;
              if (s < 0.0) {
                s = 0.0;
              }
              temp2 = temp / vn2[ii - 1];
              temp2 = s * (temp2 * temp2);
              if (temp2 <= 1.4901161193847656E-8) {
                if (b_i < mrows) {
                  temp = internal::blas::xnrm2(mmi, obj->QR, ij + 1);
                  vn1[ii - 1] = temp;
                  vn2[ii - 1] = temp;
                } else {
                  vn1[ii - 1] = 0.0;
                  vn2[ii - 1] = 0.0;
                }
              } else {
                vn1[ii - 1] = temp * std::sqrt(s);
              }
            }
          }
        }
      }
    }
  }
}

} // namespace QRManager
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for factorQRE.cpp
//
// [EOF]
//
