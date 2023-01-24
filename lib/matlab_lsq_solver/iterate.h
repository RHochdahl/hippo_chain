//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: iterate.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

#ifndef ITERATE_H
#define ITERATE_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct struct_T;

struct e_struct_T;

struct f_struct_T;

struct c_struct_T;

struct d_struct_T;

struct b_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void iterate(const ::coder::array<double, 2U> &H,
             const ::coder::array<double, 1U> &f, struct_T *solution,
             e_struct_T *memspace, f_struct_T *workingset,
             c_struct_T *qrmanager, d_struct_T *cholmanager,
             b_struct_T *objective, double options_ObjectiveLimit,
             double options_StepTolerance, int runTimeOptions_MaxIterations,
             double runTimeOptions_ProbRelTolFactor,
             bool runTimeOptions_RemainFeasible);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for iterate.h
//
// [EOF]
//
