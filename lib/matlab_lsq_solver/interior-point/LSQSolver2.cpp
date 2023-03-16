//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: LSQSolver2.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 14-Mar-2023 23:05:22
//

// Include Files
#include "LSQSolver2.h"
#include "qpipmw.h"
#include "../rt_nonfinite.h"
#include "sum.h"
#include "trisolve.h"
#include "unsafeSxfun.h"
#include "../coder_array.h"
#include <cmath>

// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

//
// Arguments    : void
// Return Type  : void
//
LSQSolver2::LSQSolver2() = default;

//
// Arguments    : void
// Return Type  : void
//
LSQSolver2::~LSQSolver2() = default;

//
// Arguments    : const coder::array<double, 2U> &Q
//                const coder::array<double, 1U> &q
//                const coder::array<double, 1U> &x0
//                coder::array<double, 1U> &x
// Return Type  : void
//
void LSQSolver2::solveLSQ2(const coder::array<double, 2U> &Q,
                          const coder::array<double, 1U> &q,
                          const coder::array<double, 1U> &x0,
                          coder::array<double, 1U> &x)
{
  coder::array<double, 2U> A;
  coder::array<double, 2U> E;
  coder::array<double, 2U> b_I;
  coder::array<double, 2U> pp;
  coder::array<double, 2U> varargin_1;
  coder::array<double, 1U> b;
  coder::array<double, 1U> b_b;
  coder::array<double, 1U> ds;
  coder::array<double, 1U> ds1;
  coder::array<double, 1U> dx;
  coder::array<double, 1U> dz;
  coder::array<double, 1U> dz1;
  coder::array<double, 1U> r;
  coder::array<double, 1U> r1;
  coder::array<double, 1U> r2;
  coder::array<double, 1U> rS;
  coder::array<double, 1U> s;
  coder::array<double, 1U> z;
  coder::array<boolean_T, 1U> b_x;
  double varargin_2;
  int sizes[2];
  int i;
  int input_sizes_idx_0;
  int iter;
  int m;
  int minszA;
  int nx;
  boolean_T empty_non_axis_sizes;
  m = x0.size(0);
  varargin_1.set_size(x0.size(0), x0.size(0));
  nx = x0.size(0) * x0.size(0);
  for (i = 0; i < nx; i++) {
    varargin_1[i] = 0.0;
  }
  if (x0.size(0) > 0) {
    for (input_sizes_idx_0 = 0; input_sizes_idx_0 < m; input_sizes_idx_0++) {
      varargin_1[input_sizes_idx_0 + varargin_1.size(0) * input_sizes_idx_0] =
          1.0;
    }
  }
  m = x0.size(0);
  b_I.set_size(x0.size(0), x0.size(0));
  nx = x0.size(0) * x0.size(0);
  for (i = 0; i < nx; i++) {
    b_I[i] = 0.0;
  }
  if (x0.size(0) > 0) {
    for (input_sizes_idx_0 = 0; input_sizes_idx_0 < m; input_sizes_idx_0++) {
      b_I[input_sizes_idx_0 + b_I.size(0) * input_sizes_idx_0] = 1.0;
    }
  }
  nx = b_I.size(0) * b_I.size(1);
  for (i = 0; i < nx; i++) {
    b_I[i] = -b_I[i];
  }
  if ((varargin_1.size(0) != 0) && (varargin_1.size(1) != 0)) {
    minszA = varargin_1.size(1);
  } else if ((b_I.size(0) != 0) && (b_I.size(1) != 0)) {
    minszA = b_I.size(1);
  } else {
    minszA = varargin_1.size(1);
    if (b_I.size(1) > varargin_1.size(1)) {
      minszA = b_I.size(1);
    }
  }
  empty_non_axis_sizes = (minszA == 0);
  if (empty_non_axis_sizes ||
      ((varargin_1.size(0) != 0) && (varargin_1.size(1) != 0))) {
    input_sizes_idx_0 = varargin_1.size(0);
  } else {
    input_sizes_idx_0 = 0;
  }
  if (empty_non_axis_sizes || ((b_I.size(0) != 0) && (b_I.size(1) != 0))) {
    sizes[0] = b_I.size(0);
  } else {
    sizes[0] = 0;
  }
  b.set_size(2 * x0.size(0));
  nx = 2 * x0.size(0);
  for (i = 0; i < nx; i++) {
    b[i] = 1.0;
  }
  nx = input_sizes_idx_0;
  input_sizes_idx_0 = sizes[0];
  A.set_size(nx + sizes[0], minszA);
  for (i = 0; i < minszA; i++) {
    for (m = 0; m < nx; m++) {
      A[m + A.size(0) * i] = varargin_1[m + nx * i];
    }
  }
  for (i = 0; i < minszA; i++) {
    for (m = 0; m < input_sizes_idx_0; m++) {
      A[(m + nx) + A.size(0) * i] = b_I[m + input_sizes_idx_0 * i];
    }
  }
  E.set_size(0, x0.size(0));
  m = A.size(0);
  iter = 0;
  x.set_size(x0.size(0));
  nx = x0.size(0);
  for (i = 0; i < nx; i++) {
    x[i] = x0[i];
  }
  z.set_size(A.size(0));
  nx = A.size(0);
  for (i = 0; i < nx; i++) {
    z[i] = 1.0;
  }
  s.set_size(A.size(0));
  nx = A.size(0);
  for (i = 0; i < nx; i++) {
    s[i] = 1.0;
  }
  if (A.size(0) == 0) {
    coder::kkt_fact(Q, A, E, z, s, varargin_1, b_I, pp);
    sizes[0] = A.size(1);
    sizes[1] = 0;
    if (q.size(0) == A.size(1)) {
      b_b.set_size(q.size(0));
      nx = q.size(0);
      for (i = 0; i < nx; i++) {
        b_b[i] = -q[i];
      }
    } else {
      binary_expand_op(b_b, q, sizes);
    }
    input_sizes_idx_0 = varargin_1.size(1);
    nx = varargin_1.size(0);
    minszA = varargin_1.size(1);
    if (nx <= minszA) {
      minszA = nx;
    }
    dz1.set_size(varargin_1.size(1));
    for (nx = 0; nx < minszA; nx++) {
      dz1[nx] = b_b[static_cast<int>(pp[nx]) - 1];
    }
    i = minszA + 1;
    for (nx = i; nx <= input_sizes_idx_0; nx++) {
      dz1[nx - 1] = 0.0;
    }
    coder::internal::trisolve(varargin_1, dz1);
    input_sizes_idx_0 = b_I.size(1);
    nx = b_I.size(0);
    minszA = b_I.size(1);
    if (nx <= minszA) {
      minszA = nx;
    }
    b_b.set_size(b_I.size(1));
    for (nx = 0; nx < minszA; nx++) {
      b_b[nx] = dz1[nx];
    }
    i = minszA + 1;
    for (nx = i; nx <= input_sizes_idx_0; nx++) {
      b_b[nx - 1] = 0.0;
    }
    coder::internal::b_trisolve(b_I, b_b);
    if (A.size(1) < 1) {
      nx = 0;
    } else {
      nx = A.size(1);
    }
    x.set_size(nx);
    for (i = 0; i < nx; i++) {
      x[i] = b_b[i];
    }
  } else {
    coder::kkt_residual(Q, q, A, b, E, x0, z, s, dz1, ds1, rS);
    coder::kkt_fact(Q, A, E, z, s, varargin_1, b_I, pp);
    coder::kkt_solve(A, dz1, ds1, rS, z, s, varargin_1, b_I, pp, b_b, dz, ds);
    if (A.size(0) == dz.size(0)) {
      z.set_size(A.size(0));
      nx = A.size(0);
      for (i = 0; i < nx; i++) {
        varargin_2 = -1.0 - dz[i];
        z[i] = std::fmax(0.0, varargin_2);
      }
    } else {
      b_binary_expand_op(z, A, dz);
    }
    if (A.size(0) == 1) {
      i = dz.size(0);
    } else {
      i = A.size(0);
    }
    if ((A.size(0) == dz.size(0)) && (i == z.size(0))) {
      nx = A.size(0);
      z.set_size(A.size(0));
      for (i = 0; i < nx; i++) {
        z[i] = ((dz[i] + 1.0) + 0.1) + z[i];
      }
    } else {
      binary_expand_op(z, A, dz);
    }
    if (A.size(0) == ds.size(0)) {
      s.set_size(A.size(0));
      nx = A.size(0);
      for (i = 0; i < nx; i++) {
        varargin_2 = -1.0 - ds[i];
        s[i] = std::fmax(0.0, varargin_2);
      }
    } else {
      b_binary_expand_op(s, A, ds);
    }
    if (A.size(0) == 1) {
      i = ds.size(0);
    } else {
      i = A.size(0);
    }
    if ((A.size(0) == ds.size(0)) && (i == s.size(0))) {
      nx = A.size(0);
      s.set_size(A.size(0));
      for (i = 0; i < nx; i++) {
        s[i] = ((ds[i] + 1.0) + 0.1) + s[i];
      }
    } else {
      binary_expand_op(s, A, ds);
    }
  }
  if (A.size(0) != 0) {
    boolean_T exitg1;
    dz.set_size(A.size(0));
    nx = A.size(0);
    for (i = 0; i < nx; i++) {
      dz[i] = 1.0;
    }
    exitg1 = false;
    while ((!exitg1) && (iter < 50)) {
      double mu;
      boolean_T exitg2;
      boolean_T guard1{false};
      coder::kkt_residual(Q, q, A, b, E, x, z, s, dz1, ds1, rS);
      mu = -coder::sum(rS) / static_cast<double>(m);
      b_x.set_size(ds1.size(0));
      nx = ds1.size(0);
      for (i = 0; i < nx; i++) {
        b_x[i] = (ds1[i] <= 1.0E-6);
      }
      empty_non_axis_sizes = true;
      nx = 1;
      exitg2 = false;
      while ((!exitg2) && (nx <= b_x.size(0))) {
        if (!b_x[nx - 1]) {
          empty_non_axis_sizes = false;
          exitg2 = true;
        } else {
          nx++;
        }
      }
      guard1 = false;
      if (empty_non_axis_sizes) {
        nx = dz1.size(0);
        b_b.set_size(dz1.size(0));
        for (input_sizes_idx_0 = 0; input_sizes_idx_0 < nx;
             input_sizes_idx_0++) {
          b_b[input_sizes_idx_0] = std::abs(dz1[input_sizes_idx_0]);
        }
        b_x.set_size(b_b.size(0));
        nx = b_b.size(0);
        for (i = 0; i < nx; i++) {
          b_x[i] = (b_b[i] <= 1.0E-6);
        }
        empty_non_axis_sizes = true;
        nx = 1;
        exitg2 = false;
        while ((!exitg2) && (nx <= b_x.size(0))) {
          if (!b_x[nx - 1]) {
            empty_non_axis_sizes = false;
            exitg2 = true;
          } else {
            nx++;
          }
        }
        if (empty_non_axis_sizes && (mu <= 1.0E-8)) {
          exitg1 = true;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      if (guard1) {
        nx = dz.size(0);
        b_b.set_size(dz.size(0));
        for (input_sizes_idx_0 = 0; input_sizes_idx_0 < nx;
             input_sizes_idx_0++) {
          b_b[input_sizes_idx_0] = std::abs(dz[input_sizes_idx_0]);
        }
        b_x.set_size(b_b.size(0));
        nx = b_b.size(0);
        for (i = 0; i < nx; i++) {
          b_x[i] = (b_b[i] <= 1.0E-8);
        }
        empty_non_axis_sizes = true;
        nx = 1;
        exitg2 = false;
        while ((!exitg2) && (nx <= b_x.size(0))) {
          if (!b_x[nx - 1]) {
            empty_non_axis_sizes = false;
            exitg2 = true;
          } else {
            nx++;
          }
        }
        if (empty_non_axis_sizes) {
          exitg1 = true;
        } else {
          double as;
          double az;
          iter++;
          coder::kkt_fact(Q, A, E, z, s, varargin_1, b_I, pp);
          coder::kkt_solve(A, dz1, ds1, rS, z, s, varargin_1, b_I, pp, dx, dz,
                           ds);
          az = 1.0;
          i = dz.size(0);
          for (nx = 0; nx < i; nx++) {
            if (dz[nx] < 0.0) {
              az = std::fmin(az, (1.0E-8 - z[nx]) / dz[nx]);
            }
          }
          as = 1.0;
          i = ds.size(0);
          for (nx = 0; nx < i; nx++) {
            if (ds[nx] < 0.0) {
              as = std::fmin(as, (1.0E-8 - s[nx]) / ds[nx]);
            }
          }
          as = std::fmin(az, as);
          az = as;
          input_sizes_idx_0 = 0;
          empty_non_axis_sizes = false;
          while ((input_sizes_idx_0 < 5) && (!empty_non_axis_sizes)) {
            double as1;
            double b_varargin_2;
            double sigma;
            input_sizes_idx_0++;
            if (z.size(0) == 1) {
              i = dz.size(0);
            } else {
              i = z.size(0);
            }
            if (s.size(0) == 1) {
              minszA = ds.size(0);
            } else {
              minszA = s.size(0);
            }
            if ((z.size(0) == dz.size(0)) && (s.size(0) == ds.size(0)) &&
                (i == minszA)) {
              rS.set_size(z.size(0));
              nx = z.size(0);
              for (i = 0; i < nx; i++) {
                rS[i] = -(z[i] + az * dz[i]) * (s[i] + as * ds[i]);
              }
            } else {
              binary_expand_op(rS, z, az, dz, s, as, ds);
            }
            sigma =
                rt_powd_snf(-coder::sum(rS) / static_cast<double>(m) / mu, 3.0);
            nx = rS.size(0);
            for (i = 0; i < nx; i++) {
              rS[i] = -rS[i];
            }
            varargin_2 = 10.0 * sigma * mu;
            b_varargin_2 = sigma * mu;
            as1 = -10.0 * sigma * mu;
            r.set_size(rS.size(0));
            nx = rS.size(0);
            for (i = 0; i < nx; i++) {
              sigma = rS[i];
              r[i] = std::fmin(sigma, varargin_2);
            }
            nx = r.size(0);
            for (i = 0; i < nx; i++) {
              sigma = r[i];
              r[i] = std::fmax(sigma, b_varargin_2);
            }
            if (r.size(0) == rS.size(0)) {
              nx = r.size(0);
              for (i = 0; i < nx; i++) {
                sigma = r[i] - rS[i];
                r[i] = std::fmax(sigma, as1);
              }
            } else {
              binary_expand_op(r, rS, as1);
            }
            r1.set_size(A.size(1));
            nx = A.size(1);
            for (i = 0; i < nx; i++) {
              r1[i] = 0.0;
            }
            r2.set_size(m);
            for (i = 0; i < m; i++) {
              r2[i] = 0.0;
            }
            coder::kkt_solve(A, r1, r2, r, z, s, varargin_1, b_I, pp, b_b, dz1,
                             ds1);
            if (dz.size(0) == dz1.size(0)) {
              dz1.set_size(dz.size(0));
              nx = dz.size(0);
              for (i = 0; i < nx; i++) {
                dz1[i] = dz[i] + dz1[i];
              }
            } else {
              b_plus(dz1, dz);
            }
            if (ds.size(0) == ds1.size(0)) {
              ds1.set_size(ds.size(0));
              nx = ds.size(0);
              for (i = 0; i < nx; i++) {
                ds1[i] = ds[i] + ds1[i];
              }
            } else {
              b_plus(ds1, ds);
            }
            sigma = 1.0;
            i = dz1.size(0);
            for (nx = 0; nx < i; nx++) {
              if (dz1[nx] < 0.0) {
                sigma = std::fmin(sigma, (1.0E-8 - z[nx]) / dz1[nx]);
              }
            }
            as1 = 1.0;
            i = ds1.size(0);
            for (nx = 0; nx < i; nx++) {
              if (ds1[nx] < 0.0) {
                as1 = std::fmin(as1, (1.0E-8 - s[nx]) / ds1[nx]);
              }
            }
            as1 = std::fmin(sigma, as1);
            if ((input_sizes_idx_0 > 1) &&
                ((as1 < az + 0.010000000000000002) ||
                 (as1 < as + 0.010000000000000002))) {
              empty_non_axis_sizes = true;
            } else {
              az = as1;
              as = as1;
              if (dx.size(0) == b_b.size(0)) {
                nx = dx.size(0);
                for (i = 0; i < nx; i++) {
                  dx[i] = dx[i] + b_b[i];
                }
              } else {
                plus(dx, b_b);
              }
              ds.set_size(ds1.size(0));
              nx = ds1.size(0);
              for (i = 0; i < nx; i++) {
                ds[i] = ds1[i];
              }
              dz.set_size(dz1.size(0));
              nx = dz1.size(0);
              for (i = 0; i < nx; i++) {
                dz[i] = dz1[i];
              }
            }
          }
          as = std::fmin(0.99995 * az, 0.99995 * as);
          nx = dx.size(0);
          for (i = 0; i < nx; i++) {
            dx[i] = as * dx[i];
          }
          nx = dz.size(0);
          for (i = 0; i < nx; i++) {
            dz[i] = as * dz[i];
          }
          nx = ds.size(0);
          for (i = 0; i < nx; i++) {
            ds[i] = as * ds[i];
          }
          if (x.size(0) == dx.size(0)) {
            nx = x.size(0);
            for (i = 0; i < nx; i++) {
              x[i] = x[i] + dx[i];
            }
          } else {
            plus(x, dx);
          }
          if (z.size(0) == dz.size(0)) {
            nx = z.size(0);
            for (i = 0; i < nx; i++) {
              z[i] = z[i] + dz[i];
            }
          } else {
            plus(z, dz);
          }
          if (s.size(0) == ds.size(0)) {
            nx = s.size(0);
            for (i = 0; i < nx; i++) {
              s[i] = s[i] + ds[i];
            }
          } else {
            plus(s, ds);
          }
        }
      }
    }
  }
}

//
// File trailer for LSQSolver2.cpp
//
// [EOF]
//
