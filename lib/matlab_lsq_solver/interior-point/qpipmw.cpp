//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: qpipmw.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 14-Mar-2023 23:05:22
//

// Include Files
#include "qpipmw.h"
#include "div.h"
#include "../rt_nonfinite.h"
#include "trisolve.h"
#include "../coder_array.h"
#include <cmath>

// Function Declarations
static void b_binary_expand_op(coder::array<double, 1U> &in1,
                               const coder::array<double, 1U> &in2,
                               const coder::array<double, 1U> &in3);

static void b_binary_expand_op(coder::array<double, 1U> &in1,
                               const coder::array<double, 1U> &in2,
                               const coder::array<double, 1U> &in3,
                               const coder::array<double, 1U> &in4,
                               const coder::array<double, 1U> &in5);

static void binary_expand_op(coder::array<double, 1U> &in1,
                             const coder::array<double, 2U> &in2,
                             const coder::array<double, 1U> &in3,
                             const coder::array<double, 1U> &in4);

static void binary_expand_op(coder::array<double, 1U> &in1,
                             const coder::array<double, 1U> &in2,
                             const coder::array<double, 1U> &in3);

static void minus(coder::array<double, 1U> &in1,
                  const coder::array<double, 1U> &in2);

static void plus(coder::array<double, 2U> &in1,
                 const coder::array<double, 2U> &in2);

// Function Definitions
//
// Arguments    : coder::array<double, 1U> &in1
//                const coder::array<double, 1U> &in2
//                const coder::array<double, 1U> &in3
// Return Type  : void
//
static void b_binary_expand_op(coder::array<double, 1U> &in1,
                               const coder::array<double, 1U> &in2,
                               const coder::array<double, 1U> &in3)
{
  coder::array<double, 1U> b_in1;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  int stride_2_0;
  if (in3.size(0) == 1) {
    if (in2.size(0) == 1) {
      i = in1.size(0);
    } else {
      i = in2.size(0);
    }
  } else {
    i = in3.size(0);
  }
  b_in1.set_size(i);
  stride_0_0 = (in1.size(0) != 1);
  stride_1_0 = (in2.size(0) != 1);
  stride_2_0 = (in3.size(0) != 1);
  if (in3.size(0) == 1) {
    if (in2.size(0) == 1) {
      loop_ub = in1.size(0);
    } else {
      loop_ub = in2.size(0);
    }
  } else {
    loop_ub = in3.size(0);
  }
  for (i = 0; i < loop_ub; i++) {
    b_in1[i] =
        (in1[i * stride_0_0] + in2[i * stride_1_0]) - in3[i * stride_2_0];
  }
  in1.set_size(b_in1.size(0));
  loop_ub = b_in1.size(0);
  for (i = 0; i < loop_ub; i++) {
    in1[i] = b_in1[i];
  }
}

//
// Arguments    : coder::array<double, 1U> &in1
//                const coder::array<double, 1U> &in2
//                const coder::array<double, 1U> &in3
//                const coder::array<double, 1U> &in4
//                const coder::array<double, 1U> &in5
// Return Type  : void
//
static void b_binary_expand_op(coder::array<double, 1U> &in1,
                               const coder::array<double, 1U> &in2,
                               const coder::array<double, 1U> &in3,
                               const coder::array<double, 1U> &in4,
                               const coder::array<double, 1U> &in5)
{
  coder::array<double, 1U> b_in2;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  int stride_2_0;
  int stride_3_0;
  int stride_4_0;
  int stride_5_0;
  if (in3.size(0) == 1) {
    i = in5.size(0);
  } else {
    i = in3.size(0);
  }
  if (in4.size(0) == 1) {
    stride_0_0 = in1.size(0);
  } else {
    stride_0_0 = in4.size(0);
  }
  if (i == 1) {
    if (stride_0_0 == 1) {
      if (in3.size(0) == 1) {
        i = in2.size(0);
      } else {
        i = in3.size(0);
      }
    } else if (in4.size(0) == 1) {
      i = in1.size(0);
    } else {
      i = in4.size(0);
    }
  } else if (in3.size(0) == 1) {
    i = in5.size(0);
  } else {
    i = in3.size(0);
  }
  b_in2.set_size(i);
  stride_0_0 = (in2.size(0) != 1);
  stride_1_0 = (in3.size(0) != 1);
  stride_2_0 = (in1.size(0) != 1);
  stride_3_0 = (in4.size(0) != 1);
  stride_4_0 = (in5.size(0) != 1);
  stride_5_0 = (in3.size(0) != 1);
  if (in3.size(0) == 1) {
    i = in5.size(0);
  } else {
    i = in3.size(0);
  }
  if (i == 1) {
    if (in4.size(0) == 1) {
      i = in1.size(0);
    } else {
      i = in4.size(0);
    }
    if (i == 1) {
      if (in3.size(0) == 1) {
        loop_ub = in2.size(0);
      } else {
        loop_ub = in3.size(0);
      }
    } else if (in4.size(0) == 1) {
      loop_ub = in1.size(0);
    } else {
      loop_ub = in4.size(0);
    }
  } else if (in3.size(0) == 1) {
    loop_ub = in5.size(0);
  } else {
    loop_ub = in3.size(0);
  }
  for (i = 0; i < loop_ub; i++) {
    b_in2[i] = in2[i * stride_0_0] / in3[i * stride_1_0] *
                   (in1[i * stride_2_0] + in4[i * stride_3_0]) +
               in5[i * stride_4_0] / in3[i * stride_5_0];
  }
  in1.set_size(b_in2.size(0));
  loop_ub = b_in2.size(0);
  for (i = 0; i < loop_ub; i++) {
    in1[i] = b_in2[i];
  }
}

//
// Arguments    : coder::array<double, 1U> &in1
//                const coder::array<double, 2U> &in2
//                const coder::array<double, 1U> &in3
//                const coder::array<double, 1U> &in4
// Return Type  : void
//
static void binary_expand_op(coder::array<double, 1U> &in1,
                             const coder::array<double, 2U> &in2,
                             const coder::array<double, 1U> &in3,
                             const coder::array<double, 1U> &in4)
{
  coder::array<double, 1U> b_in1;
  int i;
  int in2_idx_0;
  int stride_0_0;
  int stride_2_0;
  int stride_3_0;
  in2_idx_0 = in2.size(1);
  if (in4.size(0) == 1) {
    if (in3.size(0) == 1) {
      if (in2_idx_0 == 1) {
        i = in1.size(0);
      } else {
        i = in2_idx_0;
      }
    } else {
      i = in3.size(0);
    }
  } else {
    i = in4.size(0);
  }
  b_in1.set_size(i);
  stride_0_0 = (in1.size(0) != 1);
  stride_2_0 = (in3.size(0) != 1);
  stride_3_0 = (in4.size(0) != 1);
  if (in4.size(0) == 1) {
    if (in3.size(0) == 1) {
      if (in2_idx_0 == 1) {
        in2_idx_0 = in1.size(0);
      }
    } else {
      in2_idx_0 = in3.size(0);
    }
  } else {
    in2_idx_0 = in4.size(0);
  }
  for (i = 0; i < in2_idx_0; i++) {
    b_in1[i] =
        -((in1[i * stride_0_0] + in3[i * stride_2_0]) + in4[i * stride_3_0]);
  }
  in1.set_size(b_in1.size(0));
  in2_idx_0 = b_in1.size(0);
  for (i = 0; i < in2_idx_0; i++) {
    in1[i] = b_in1[i];
  }
}

//
// Arguments    : coder::array<double, 1U> &in1
//                const coder::array<double, 1U> &in2
//                const coder::array<double, 1U> &in3
// Return Type  : void
//
static void binary_expand_op(coder::array<double, 1U> &in1,
                             const coder::array<double, 1U> &in2,
                             const coder::array<double, 1U> &in3)
{
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  if (in3.size(0) == 1) {
    i = in2.size(0);
  } else {
    i = in3.size(0);
  }
  in1.set_size(i);
  stride_0_0 = (in2.size(0) != 1);
  stride_1_0 = (in3.size(0) != 1);
  if (in3.size(0) == 1) {
    loop_ub = in2.size(0);
  } else {
    loop_ub = in3.size(0);
  }
  for (i = 0; i < loop_ub; i++) {
    in1[i] = -in2[i * stride_0_0] * in3[i * stride_1_0];
  }
}

//
// Arguments    : coder::array<double, 1U> &in1
//                const coder::array<double, 1U> &in2
// Return Type  : void
//
static void minus(coder::array<double, 1U> &in1,
                  const coder::array<double, 1U> &in2)
{
  coder::array<double, 1U> b_in2;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  if (in1.size(0) == 1) {
    i = in2.size(0);
  } else {
    i = in1.size(0);
  }
  b_in2.set_size(i);
  stride_0_0 = (in2.size(0) != 1);
  stride_1_0 = (in1.size(0) != 1);
  if (in1.size(0) == 1) {
    loop_ub = in2.size(0);
  } else {
    loop_ub = in1.size(0);
  }
  for (i = 0; i < loop_ub; i++) {
    b_in2[i] = in2[i * stride_0_0] - in1[i * stride_1_0];
  }
  in1.set_size(b_in2.size(0));
  loop_ub = b_in2.size(0);
  for (i = 0; i < loop_ub; i++) {
    in1[i] = b_in2[i];
  }
}

//
// Arguments    : coder::array<double, 2U> &in1
//                const coder::array<double, 2U> &in2
// Return Type  : void
//
static void plus(coder::array<double, 2U> &in1,
                 const coder::array<double, 2U> &in2)
{
  coder::array<double, 2U> b_in2;
  int aux_0_1;
  int aux_1_1;
  int b_loop_ub;
  int i;
  int i1;
  int loop_ub;
  int stride_0_0;
  int stride_0_1;
  int stride_1_0;
  int stride_1_1;
  if (in1.size(0) == 1) {
    i = in2.size(0);
  } else {
    i = in1.size(0);
  }
  if (in1.size(1) == 1) {
    i1 = in2.size(1);
  } else {
    i1 = in1.size(1);
  }
  b_in2.set_size(i, i1);
  stride_0_0 = (in2.size(0) != 1);
  stride_0_1 = (in2.size(1) != 1);
  stride_1_0 = (in1.size(0) != 1);
  stride_1_1 = (in1.size(1) != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  if (in1.size(1) == 1) {
    loop_ub = in2.size(1);
  } else {
    loop_ub = in1.size(1);
  }
  for (i = 0; i < loop_ub; i++) {
    if (in1.size(0) == 1) {
      b_loop_ub = in2.size(0);
    } else {
      b_loop_ub = in1.size(0);
    }
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      b_in2[i1 + b_in2.size(0) * i] =
          in2[i1 * stride_0_0 + in2.size(0) * aux_0_1] +
          in1[i1 * stride_1_0 + in1.size(0) * aux_1_1];
    }
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  in1.set_size(b_in2.size(0), b_in2.size(1));
  loop_ub = b_in2.size(1);
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = b_in2.size(0);
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      in1[i1 + in1.size(0) * i] = b_in2[i1 + b_in2.size(0) * i];
    }
  }
}

//
// Arguments    : coder::array<double, 1U> &in1
//                const coder::array<double, 1U> &in2
// Return Type  : void
//
void b_plus(coder::array<double, 1U> &in1, const coder::array<double, 1U> &in2)
{
  coder::array<double, 1U> b_in2;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  if (in1.size(0) == 1) {
    i = in2.size(0);
  } else {
    i = in1.size(0);
  }
  b_in2.set_size(i);
  stride_0_0 = (in2.size(0) != 1);
  stride_1_0 = (in1.size(0) != 1);
  if (in1.size(0) == 1) {
    loop_ub = in2.size(0);
  } else {
    loop_ub = in1.size(0);
  }
  for (i = 0; i < loop_ub; i++) {
    b_in2[i] = in2[i * stride_0_0] + in1[i * stride_1_0];
  }
  in1.set_size(b_in2.size(0));
  loop_ub = b_in2.size(0);
  for (i = 0; i < loop_ub; i++) {
    in1[i] = b_in2[i];
  }
}

//
// Arguments    : coder::array<double, 1U> &in1
//                const coder::array<double, 2U> &in2
//                const coder::array<double, 1U> &in3
// Return Type  : void
//
void binary_expand_op(coder::array<double, 1U> &in1,
                      const coder::array<double, 2U> &in2,
                      const coder::array<double, 1U> &in3)
{
  coder::array<double, 1U> r;
  int i;
  int in2_idx_0;
  int stride_1_0;
  int stride_2_0;
  in2_idx_0 = in2.size(0);
  if (in1.size(0) == 1) {
    if (in3.size(0) == 1) {
      i = in2_idx_0;
    } else {
      i = in3.size(0);
    }
  } else {
    i = in1.size(0);
  }
  r.set_size(i);
  stride_1_0 = (in3.size(0) != 1);
  stride_2_0 = (in1.size(0) != 1);
  if (in1.size(0) == 1) {
    if (in3.size(0) != 1) {
      in2_idx_0 = in3.size(0);
    }
  } else {
    in2_idx_0 = in1.size(0);
  }
  for (i = 0; i < in2_idx_0; i++) {
    r[i] = ((in3[i * stride_1_0] + 1.0) + 0.1) + in1[i * stride_2_0];
  }
  in1.set_size(r.size(0));
  in2_idx_0 = r.size(0);
  for (i = 0; i < in2_idx_0; i++) {
    in1[i] = r[i];
  }
}

//
// Arguments    : coder::array<double, 1U> &in1
//                const coder::array<double, 1U> &in2
//                const int in3[2]
// Return Type  : void
//
void binary_expand_op(coder::array<double, 1U> &in1,
                      const coder::array<double, 1U> &in2, const int in3[2])
{
  int b_in3_idx_0;
  int in3_idx_0;
  in3_idx_0 = in3[0];
  if (in3_idx_0 == 1) {
    b_in3_idx_0 = in2.size(0);
  } else {
    b_in3_idx_0 = in3_idx_0;
  }
  in1.set_size(b_in3_idx_0);
  b_in3_idx_0 = (in2.size(0) != 1);
  if (in3_idx_0 == 1) {
    in3_idx_0 = in2.size(0);
  }
  for (int i{0}; i < in3_idx_0; i++) {
    in1[i] = -in2[i * b_in3_idx_0];
  }
}

//
// Arguments    : coder::array<double, 1U> &in1
//                const coder::array<double, 1U> &in2
//                double in3
//                const coder::array<double, 1U> &in4
//                const coder::array<double, 1U> &in5
//                double in6
//                const coder::array<double, 1U> &in7
// Return Type  : void
//
void binary_expand_op(coder::array<double, 1U> &in1,
                      const coder::array<double, 1U> &in2, double in3,
                      const coder::array<double, 1U> &in4,
                      const coder::array<double, 1U> &in5, double in6,
                      const coder::array<double, 1U> &in7)
{
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  int stride_2_0;
  int stride_3_0;
  if (in7.size(0) == 1) {
    i = in5.size(0);
  } else {
    i = in7.size(0);
  }
  if (i == 1) {
    if (in4.size(0) == 1) {
      i = in2.size(0);
    } else {
      i = in4.size(0);
    }
  } else if (in7.size(0) == 1) {
    i = in5.size(0);
  } else {
    i = in7.size(0);
  }
  in1.set_size(i);
  stride_0_0 = (in2.size(0) != 1);
  stride_1_0 = (in4.size(0) != 1);
  stride_2_0 = (in5.size(0) != 1);
  stride_3_0 = (in7.size(0) != 1);
  if (in7.size(0) == 1) {
    i = in5.size(0);
  } else {
    i = in7.size(0);
  }
  if (i == 1) {
    if (in4.size(0) == 1) {
      loop_ub = in2.size(0);
    } else {
      loop_ub = in4.size(0);
    }
  } else if (in7.size(0) == 1) {
    loop_ub = in5.size(0);
  } else {
    loop_ub = in7.size(0);
  }
  for (i = 0; i < loop_ub; i++) {
    in1[i] = -(in2[i * stride_0_0] + in3 * in4[i * stride_1_0]) *
             (in5[i * stride_2_0] + in6 * in7[i * stride_3_0]);
  }
}

//
// Arguments    : const ::coder::array<double, 2U> &Q
//                const ::coder::array<double, 2U> &A
//                const ::coder::array<double, 2U> &E
//                const ::coder::array<double, 1U> &z
//                const ::coder::array<double, 1U> &s
//                ::coder::array<double, 2U> &L
//                ::coder::array<double, 2U> &U
//                ::coder::array<double, 2U> &pp
// Return Type  : void
//
namespace coder {
void kkt_fact(const ::coder::array<double, 2U> &Q,
              const ::coder::array<double, 2U> &A,
              const ::coder::array<double, 2U> &E,
              const ::coder::array<double, 1U> &z,
              const ::coder::array<double, 1U> &s,
              ::coder::array<double, 2U> &L, ::coder::array<double, 2U> &U,
              ::coder::array<double, 2U> &pp)
{
  array<double, 2U> b;
  array<double, 2U> varargin_1;
  array<double, 2U> y;
  array<double, 1U> b_z;
  array<int, 2U> ipiv;
  array<int, 2U> perm;
  double bkj;
  int aoffset;
  int b_i;
  unsigned int b_j;
  int boffset;
  int coffset;
  int i;
  int jA;
  int jy;
  int m;
  int n;
  int nv;
  int u1;
  if (z.size(0) == s.size(0)) {
    b_z.set_size(z.size(0));
    jy = z.size(0);
    for (i = 0; i < jy; i++) {
      b_z[i] = z[i] / s[i];
    }
  } else {
    rdivide(b_z, z, s);
  }
  nv = b_z.size(0);
  b.set_size(b_z.size(0), b_z.size(0));
  jy = b_z.size(0) * b_z.size(0);
  for (i = 0; i < jy; i++) {
    b[i] = 0.0;
  }
  for (int j{0}; j < nv; j++) {
    b[j + b.size(0) * j] = b_z[j];
  }
  nv = A.size(1);
  jy = A.size(0);
  jA = b.size(1);
  y.set_size(A.size(1), b.size(1));
  for (int j{0}; j < jA; j++) {
    coffset = j * nv;
    boffset = j * b.size(0);
    for (b_i = 0; b_i < nv; b_i++) {
      y[coffset + b_i] = 0.0;
    }
    for (int k{0}; k < jy; k++) {
      bkj = b[boffset + k];
      for (b_i = 0; b_i < nv; b_i++) {
        i = coffset + b_i;
        y[i] = y[i] + A[b_i * A.size(0) + k] * bkj;
      }
    }
  }
  nv = y.size(0);
  jy = y.size(1);
  jA = A.size(1);
  varargin_1.set_size(y.size(0), A.size(1));
  for (int j{0}; j < jA; j++) {
    coffset = j * nv;
    boffset = j * A.size(0);
    for (b_i = 0; b_i < nv; b_i++) {
      varargin_1[coffset + b_i] = 0.0;
    }
    for (int k{0}; k < jy; k++) {
      aoffset = k * y.size(0);
      bkj = A[boffset + k];
      for (b_i = 0; b_i < nv; b_i++) {
        i = coffset + b_i;
        varargin_1[i] = varargin_1[i] + y[aoffset + b_i] * bkj;
      }
    }
  }
  if ((Q.size(0) == varargin_1.size(0)) && (Q.size(1) == varargin_1.size(1))) {
    jy = Q.size(0) * Q.size(1);
    varargin_1.set_size(Q.size(0), Q.size(1));
    for (i = 0; i < jy; i++) {
      varargin_1[i] = Q[i] + varargin_1[i];
    }
  } else {
    plus(varargin_1, Q);
  }
  if ((varargin_1.size(0) != 0) && (varargin_1.size(1) != 0)) {
    jy = varargin_1.size(0);
  } else {
    jy = varargin_1.size(0);
    if (E.size(1) > varargin_1.size(0)) {
      jy = E.size(1);
    }
  }
  if ((jy == 0) || ((varargin_1.size(0) != 0) && (varargin_1.size(1) != 0))) {
    nv = varargin_1.size(1);
  } else {
    nv = 0;
  }
  b.set_size(jy, nv);
  for (i = 0; i < nv; i++) {
    for (b_i = 0; b_i < jy; b_i++) {
      b[b_i + b.size(0) * i] = varargin_1[b_i + jy * i];
    }
  }
  if ((b.size(0) != 0) && (b.size(1) != 0)) {
    jy = b.size(1);
  } else {
    jy = b.size(1);
    if (E.size(1) > b.size(1)) {
      jy = E.size(1);
    }
  }
  if ((jy == 0) || ((b.size(0) != 0) && (b.size(1) != 0))) {
    nv = b.size(0);
  } else {
    nv = 0;
  }
  y.set_size(nv, jy);
  for (i = 0; i < jy; i++) {
    for (b_i = 0; b_i < nv; b_i++) {
      y[b_i + y.size(0) * i] = b[b_i + nv * i];
    }
  }
  m = y.size(0);
  n = y.size(1) - 2;
  b.set_size(y.size(0), y.size(1));
  jy = y.size(0) * y.size(1);
  for (i = 0; i < jy; i++) {
    b[i] = y[i];
  }
  nv = y.size(0);
  u1 = y.size(1);
  if (nv <= u1) {
    u1 = nv;
  }
  if (u1 < 1) {
    coffset = 0;
  } else {
    coffset = u1;
  }
  ipiv.set_size(1, coffset);
  if (coffset > 0) {
    ipiv[0] = 1;
    nv = 1;
    for (int k{2}; k <= coffset; k++) {
      nv++;
      ipiv[k - 1] = nv;
    }
  }
  if ((y.size(0) >= 1) && (y.size(1) >= 1)) {
    int ldap1;
    ldap1 = y.size(0);
    nv = y.size(0) - 1;
    u1 = y.size(1);
    if (nv <= u1) {
      u1 = nv;
    }
    for (int j{0}; j < u1; j++) {
      int jj;
      aoffset = m - j;
      boffset = j * (m + 1);
      jj = j * (ldap1 + 1);
      coffset = boffset + 2;
      if (aoffset < 1) {
        nv = -1;
      } else {
        nv = 0;
        if (aoffset > 1) {
          bkj = std::abs(b[jj]);
          for (int k{2}; k <= aoffset; k++) {
            double b_s;
            b_s = std::abs(b[(boffset + k) - 1]);
            if (b_s > bkj) {
              nv = k - 1;
              bkj = b_s;
            }
          }
        }
      }
      if (b[jj + nv] != 0.0) {
        if (nv != 0) {
          jy = j + nv;
          ipiv[j] = jy + 1;
          for (int k{0}; k <= n + 1; k++) {
            nv = k * m;
            jA = j + nv;
            bkj = b[jA];
            i = jy + nv;
            b[jA] = b[i];
            b[i] = bkj;
          }
        }
        i = jj + aoffset;
        for (b_i = coffset; b_i <= i; b_i++) {
          b[b_i - 1] = b[b_i - 1] / b[jj];
        }
      }
      coffset = n - j;
      jy = boffset + m;
      jA = jj + ldap1;
      for (boffset = 0; boffset <= coffset; boffset++) {
        nv = jy + boffset * m;
        bkj = b[nv];
        if (b[nv] != 0.0) {
          i = jA + 2;
          b_i = aoffset + jA;
          for (nv = i; nv <= b_i; nv++) {
            b[nv - 1] = b[nv - 1] + b[((jj + nv) - jA) - 1] * -bkj;
          }
        }
        jA += m;
      }
    }
  }
  if (y.size(0) < 1) {
    n = 0;
  } else {
    n = y.size(0);
  }
  perm.set_size(1, n);
  if (n > 0) {
    perm[0] = 1;
    nv = 1;
    for (int k{2}; k <= n; k++) {
      nv++;
      perm[k - 1] = nv;
    }
  }
  i = ipiv.size(1);
  for (int k{0}; k < i; k++) {
    b_i = ipiv[k];
    if (b_i > k + 1) {
      nv = perm[b_i - 1];
      perm[b_i - 1] = perm[k];
      perm[k] = nv;
    }
  }
  pp.set_size(1, perm.size(1));
  jy = perm.size(1);
  for (i = 0; i < jy; i++) {
    pp[i] = perm[i];
  }
  nv = static_cast<int>(std::fmin(static_cast<double>(b.size(0)),
                                  static_cast<double>(b.size(1))));
  L.set_size(b.size(0), nv);
  jy = b.size(0) * nv;
  for (i = 0; i < jy; i++) {
    L[i] = 0.0;
  }
  U.set_size(nv, b.size(1));
  jy = nv * b.size(1);
  for (i = 0; i < jy; i++) {
    U[i] = 0.0;
  }
  for (int j{0}; j < nv; j++) {
    for (b_i = 0; b_i <= j; b_i++) {
      U[b_i + U.size(0) * j] = b[b_i + b.size(0) * j];
    }
  }
  i = b.size(1) + static_cast<int>(1.0 - (static_cast<double>(nv) + 1.0));
  for (int j{0}; j < i; j++) {
    b_j = (static_cast<unsigned int>(nv) + j) + 1U;
    for (b_i = 0; b_i < nv; b_i++) {
      U[b_i + U.size(0) * (static_cast<int>(b_j) - 1)] =
          b[b_i + b.size(0) * (static_cast<int>(b_j) - 1)];
    }
  }
  for (int j{0}; j < nv; j++) {
    L[j + L.size(0) * j] = 1.0;
    i = b.size(0) - j;
    for (b_i = 0; b_i <= i - 2; b_i++) {
      b_j = (static_cast<unsigned int>(j) + b_i) + 2U;
      L[(static_cast<int>(b_j) + L.size(0) * j) - 1] =
          b[(static_cast<int>(b_j) + b.size(0) * j) - 1];
    }
  }
}

//
// Arguments    : const ::coder::array<double, 2U> &Q
//                const ::coder::array<double, 1U> &c
//                const ::coder::array<double, 2U> &A
//                const ::coder::array<double, 1U> &b
//                const ::coder::array<double, 2U> &E
//                const ::coder::array<double, 1U> &x
//                const ::coder::array<double, 1U> &z
//                const ::coder::array<double, 1U> &s
//                ::coder::array<double, 1U> &rQ
//                ::coder::array<double, 1U> &rA
//                ::coder::array<double, 1U> &rS
// Return Type  : void
//
void kkt_residual(
    const ::coder::array<double, 2U> &Q, const ::coder::array<double, 1U> &c,
    const ::coder::array<double, 2U> &A, const ::coder::array<double, 1U> &b,
    const ::coder::array<double, 2U> &E, const ::coder::array<double, 1U> &x,
    const ::coder::array<double, 1U> &z, const ::coder::array<double, 1U> &s,
    ::coder::array<double, 1U> &rQ, ::coder::array<double, 1U> &rA,
    ::coder::array<double, 1U> &rS)
{
  array<double, 1U> C;
  int aoffset;
  int inner;
  int mc;
  mc = Q.size(0) - 1;
  inner = Q.size(1);
  rQ.set_size(Q.size(0));
  for (int i{0}; i <= mc; i++) {
    rQ[i] = 0.0;
  }
  for (int k{0}; k < inner; k++) {
    aoffset = k * Q.size(0);
    for (int i{0}; i <= mc; i++) {
      rQ[i] = rQ[i] + Q[aoffset + i] * x[k];
    }
  }
  mc = A.size(1) - 1;
  inner = A.size(0);
  C.set_size(A.size(1));
  for (int i{0}; i <= mc; i++) {
    C[i] = 0.0;
  }
  for (int k{0}; k < inner; k++) {
    for (int i{0}; i <= mc; i++) {
      C[i] = C[i] + A[i * A.size(0) + k] * z[k];
    }
  }
  if (rQ.size(0) == 1) {
    mc = E.size(1);
  } else {
    mc = rQ.size(0);
  }
  if (rQ.size(0) == 1) {
    aoffset = E.size(1);
  } else {
    aoffset = rQ.size(0);
  }
  if (aoffset == 1) {
    aoffset = C.size(0);
  } else if (rQ.size(0) == 1) {
    aoffset = E.size(1);
  } else {
    aoffset = rQ.size(0);
  }
  if ((rQ.size(0) == E.size(1)) && (mc == C.size(0)) &&
      (aoffset == c.size(0))) {
    mc = rQ.size(0);
    for (inner = 0; inner < mc; inner++) {
      rQ[inner] = -((rQ[inner] + C[inner]) + c[inner]);
    }
  } else {
    binary_expand_op(rQ, E, C, c);
  }
  mc = A.size(0) - 1;
  inner = A.size(1);
  rA.set_size(A.size(0));
  for (int i{0}; i <= mc; i++) {
    rA[i] = 0.0;
  }
  for (int k{0}; k < inner; k++) {
    aoffset = k * A.size(0);
    for (int i{0}; i <= mc; i++) {
      rA[i] = rA[i] + A[aoffset + i] * x[k];
    }
  }
  if (rA.size(0) == 1) {
    mc = s.size(0);
  } else {
    mc = rA.size(0);
  }
  if ((rA.size(0) == s.size(0)) && (mc == b.size(0))) {
    mc = rA.size(0);
    for (inner = 0; inner < mc; inner++) {
      rA[inner] = (rA[inner] + s[inner]) - b[inner];
    }
  } else {
    b_binary_expand_op(rA, s, b);
  }
  if (z.size(0) == s.size(0)) {
    rS.set_size(z.size(0));
    mc = z.size(0);
    for (inner = 0; inner < mc; inner++) {
      rS[inner] = -z[inner] * s[inner];
    }
  } else {
    binary_expand_op(rS, z, s);
  }
}

//
// Arguments    : const ::coder::array<double, 2U> &A
//                const ::coder::array<double, 1U> &rQ
//                const ::coder::array<double, 1U> &rA
//                const ::coder::array<double, 1U> &rS
//                const ::coder::array<double, 1U> &z
//                const ::coder::array<double, 1U> &s
//                const ::coder::array<double, 2U> &L
//                const ::coder::array<double, 2U> &U
//                const ::coder::array<double, 2U> &pp
//                ::coder::array<double, 1U> &dx
//                ::coder::array<double, 1U> &dz
//                ::coder::array<double, 1U> &ds
// Return Type  : void
//
void kkt_solve(
    const ::coder::array<double, 2U> &A, const ::coder::array<double, 1U> &rQ,
    const ::coder::array<double, 1U> &rA, const ::coder::array<double, 1U> &rS,
    const ::coder::array<double, 1U> &z, const ::coder::array<double, 1U> &s,
    const ::coder::array<double, 2U> &L, const ::coder::array<double, 2U> &U,
    const ::coder::array<double, 2U> &pp, ::coder::array<double, 1U> &dx,
    ::coder::array<double, 1U> &dz, ::coder::array<double, 1U> &ds)
{
  array<double, 1U> B;
  array<double, 1U> b;
  int i;
  int loop_ub;
  int minszA;
  int nA;
  if (z.size(0) == 1) {
    loop_ub = rA.size(0);
    nA = rA.size(0);
  } else {
    loop_ub = z.size(0);
    nA = z.size(0);
  }
  if (nA == 1) {
    nA = rS.size(0);
  } else if (z.size(0) == 1) {
    nA = rA.size(0);
  } else {
    nA = z.size(0);
  }
  if ((z.size(0) == rA.size(0)) && (loop_ub == rS.size(0)) &&
      (nA == s.size(0))) {
    B.set_size(z.size(0));
    loop_ub = z.size(0);
    for (minszA = 0; minszA < loop_ub; minszA++) {
      B[minszA] = (z[minszA] * rA[minszA] + rS[minszA]) / s[minszA];
    }
  } else {
    c_binary_expand_op(B, z, rA, rS, s);
  }
  loop_ub = A.size(1) - 1;
  minszA = A.size(0);
  b.set_size(A.size(1));
  for (i = 0; i <= loop_ub; i++) {
    b[i] = 0.0;
  }
  for (int k{0}; k < minszA; k++) {
    for (i = 0; i <= loop_ub; i++) {
      b[i] = b[i] + A[i * A.size(0) + k] * B[k];
    }
  }
  if (rQ.size(0) == b.size(0)) {
    b.set_size(rQ.size(0));
    loop_ub = rQ.size(0);
    for (minszA = 0; minszA < loop_ub; minszA++) {
      b[minszA] = rQ[minszA] - b[minszA];
    }
  } else {
    minus(b, rQ);
  }
  nA = L.size(1);
  loop_ub = L.size(0);
  minszA = L.size(1);
  if (loop_ub <= minszA) {
    minszA = loop_ub;
  }
  B.set_size(L.size(1));
  for (i = 0; i < minszA; i++) {
    B[i] = b[static_cast<int>(pp[i]) - 1];
  }
  minszA++;
  for (i = minszA; i <= nA; i++) {
    B[i - 1] = 0.0;
  }
  internal::trisolve(L, B);
  nA = U.size(1);
  loop_ub = U.size(0);
  minszA = U.size(1);
  if (loop_ub <= minszA) {
    minszA = loop_ub;
  }
  b.set_size(U.size(1));
  for (i = 0; i < minszA; i++) {
    b[i] = B[i];
  }
  minszA++;
  for (i = minszA; i <= nA; i++) {
    b[i - 1] = 0.0;
  }
  internal::b_trisolve(U, b);
  if (A.size(1) < 1) {
    loop_ub = 0;
  } else {
    loop_ub = A.size(1);
  }
  dx.set_size(loop_ub);
  for (minszA = 0; minszA < loop_ub; minszA++) {
    dx[minszA] = b[minszA];
  }
  loop_ub = A.size(0) - 1;
  minszA = A.size(1);
  dz.set_size(A.size(0));
  for (i = 0; i <= loop_ub; i++) {
    dz[i] = 0.0;
  }
  for (int k{0}; k < minszA; k++) {
    nA = k * A.size(0);
    for (i = 0; i <= loop_ub; i++) {
      dz[i] = dz[i] + A[nA + i] * b[k];
    }
  }
  if (z.size(0) == 1) {
    loop_ub = s.size(0);
  } else {
    loop_ub = z.size(0);
  }
  if (dz.size(0) == 1) {
    nA = rA.size(0);
  } else {
    nA = dz.size(0);
  }
  if (z.size(0) == 1) {
    minszA = s.size(0);
  } else {
    minszA = z.size(0);
  }
  if (minszA == 1) {
    if (dz.size(0) == 1) {
      minszA = rA.size(0);
    } else {
      minszA = dz.size(0);
    }
  } else if (z.size(0) == 1) {
    minszA = s.size(0);
  } else {
    minszA = z.size(0);
  }
  if (rS.size(0) == 1) {
    i = s.size(0);
  } else {
    i = rS.size(0);
  }
  if ((z.size(0) == s.size(0)) && (dz.size(0) == rA.size(0)) &&
      (loop_ub == nA) && (rS.size(0) == s.size(0)) && (minszA == i)) {
    dz.set_size(z.size(0));
    loop_ub = z.size(0);
    for (minszA = 0; minszA < loop_ub; minszA++) {
      dz[minszA] = z[minszA] / s[minszA] * (dz[minszA] + rA[minszA]) +
                   rS[minszA] / s[minszA];
    }
  } else {
    b_binary_expand_op(dz, z, s, rA, rS);
  }
  if (dz.size(0) == 1) {
    loop_ub = s.size(0);
  } else {
    loop_ub = dz.size(0);
  }
  if (rS.size(0) == 1) {
    if (dz.size(0) == 1) {
      nA = s.size(0);
    } else {
      nA = dz.size(0);
    }
  } else {
    nA = rS.size(0);
  }
  if ((dz.size(0) == s.size(0)) && (rS.size(0) == loop_ub) &&
      (nA == z.size(0))) {
    ds.set_size(rS.size(0));
    loop_ub = rS.size(0);
    for (minszA = 0; minszA < loop_ub; minszA++) {
      ds[minszA] = (rS[minszA] - dz[minszA] * s[minszA]) / z[minszA];
    }
  } else {
    binary_expand_op(ds, rS, dz, s, z);
  }
}

//
// Arguments    : coder::array<double, 1U> &in1
//                const coder::array<double, 1U> &in2
// Return Type  : void
//
} // namespace coder
void plus(coder::array<double, 1U> &in1, const coder::array<double, 1U> &in2)
{
  coder::array<double, 1U> b_in1;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  if (in2.size(0) == 1) {
    i = in1.size(0);
  } else {
    i = in2.size(0);
  }
  b_in1.set_size(i);
  stride_0_0 = (in1.size(0) != 1);
  stride_1_0 = (in2.size(0) != 1);
  if (in2.size(0) == 1) {
    loop_ub = in1.size(0);
  } else {
    loop_ub = in2.size(0);
  }
  for (i = 0; i < loop_ub; i++) {
    b_in1[i] = in1[i * stride_0_0] + in2[i * stride_1_0];
  }
  in1.set_size(b_in1.size(0));
  loop_ub = b_in1.size(0);
  for (i = 0; i < loop_ub; i++) {
    in1[i] = b_in1[i];
  }
}

//
// File trailer for qpipmw.cpp
//
// [EOF]
//
