//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: PresolveWorkingSet.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 02-Jan-2023 15:29:42
//

#ifndef PRESOLVEWORKINGSET_H
#define PRESOLVEWORKINGSET_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct struct_T;

struct e_struct_T;

struct f_struct_T;

struct c_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
void PresolveWorkingSet(struct_T *solution, e_struct_T *memspace,
                        f_struct_T *workingset, c_struct_T *qrmanager);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for PresolveWorkingSet.h
//
// [EOF]
//
