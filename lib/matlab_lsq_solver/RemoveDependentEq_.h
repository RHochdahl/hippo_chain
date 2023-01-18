//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RemoveDependentEq_.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

#ifndef REMOVEDEPENDENTEQ__H
#define REMOVEDEPENDENTEQ__H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct e_struct_T;

struct f_struct_T;

struct c_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
int RemoveDependentEq_(e_struct_T *memspace, f_struct_T *workingset,
                       c_struct_T *qrmanager);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for RemoveDependentEq_.h
//
// [EOF]
//
