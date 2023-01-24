//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: factorQR.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

#ifndef FACTORQR_H
#define FACTORQR_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct c_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace QRManager {
void factorQR(c_struct_T *obj, const ::coder::array<double, 1U> &A, int mrows,
              int ncols, int ldA);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for factorQR.h
//
// [EOF]
//
