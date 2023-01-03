#ifndef LQPSOLVER_H
#define LQPSOLVER_H


#include <memory>
#include <algorithm>
#include <iostream>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <CGAL/enum.h>
#include "LeastSquaresSolver.h"


// choose exact integral type
#ifdef CGAL_USE_GMP
#include <CGAL/Gmpzf.h>
typedef CGAL::Gmpzf ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif


// solution type
typedef CGAL::Quadratic_program_solution<ET> Solution;


class LQPSolver : public LeastSquaresSolver
{
private:
    Eigen::MatrixXd Q;
    Eigen::VectorXd q;

    double** A;
    double* b;
    double** D;
    double* c;
    CGAL::Const_oneset_iterator<double> ub;
    CGAL::Const_oneset_iterator<double> lb;
    CGAL::Const_oneset_iterator<bool> bb;
    CGAL::Const_oneset_iterator<CGAL::Comparison_result> r;

    Solution s;


public:
    LQPSolver(const int size)
    : LeastSquaresSolver(size)
    , Q(Eigen::MatrixXd::Zero(SIZE, SIZE))
    , q(Eigen::VectorXd::Zero(SIZE))
    , A(new double*[0])
    , b(NULL)
    , D(new double*[SIZE])
    , c(q.data())
    , lb(-1)
    , ub(1)
    , bb(true)
    , r(CGAL::EQUAL)
    {
        D[0] = Q.data();
        for (int i=1; i<SIZE; i++) {
            D[i] = D[0] + i*SIZE;
        }
    }

    ~LQPSolver()
    {
        delete[] A;
        delete[] D;
    }

    void solve(const Eigen::Ref<const Eigen::MatrixXd>& B,
               const Eigen::Ref<const Eigen::VectorXd>& eta,
               Eigen::Ref<Eigen::VectorXd> nu)
    {
        assert(B.rows() == eta.rows());
        assert(B.cols() == SIZE);
        assert(nu.rows() == SIZE);

        Q.noalias() = B.transpose() * B;
        q.noalias() = -B.transpose() * eta;

        s = CGAL::solve_quadratic_program(CGAL::make_quadratic_program_from_iterators(SIZE, 0, A, b, r, bb, lb, bb, ub, D, c), ET());
        assert(s.variable_values_end() == s.variable_values_begin() + SIZE);

        {
            double* sol_it = nu.data();
            typename CGAL::Quadratic_program_solution<ET>::Variable_value_iterator it = s.variable_values_begin();
            for (; it != s.variable_values_end(); ++it, ++sol_it) {
                *sol_it = it->numerator().to_double() / it->denominator().to_double();
            }
        }
    }
};


#endif  // LQPSOLVER_H