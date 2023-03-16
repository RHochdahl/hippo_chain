#ifndef MATLABLSQSOLVER_H
#define MATLABLSQSOLVER_H


#include <memory>
#include <algorithm>
#include <iostream>
#include "LeastSquaresSolver.h"

#include <Eigen/Dense>

#include <cstddef>
#include <cstdlib>

// Matlab
#include <hippo_chain/lib/matlab_lsq_solver/active-set/LSQSolver.h>
#include <hippo_chain/lib/matlab_lsq_solver/rt_nonfinite.h>
#include <hippo_chain/lib/matlab_lsq_solver/rtwtypes.h>
#include <hippo_chain/lib/matlab_lsq_solver/coder_array.h>



class MatlabLSQSolver : public LeastSquaresSolver
{
private:
    LSQSolver solver;
    coder::array<double, 2U> A;
    coder::array<double, 1U> b;
    coder::array<double, 1U> x;
    coder::array<double, 1U> x0;

    Eigen::MatrixXd A_eigen;
    Eigen::VectorXd b_eigen;


    static inline coder::array<double, 1U> EigenVec2MatlabArray(Eigen::VectorXd& vec)
    {
        coder::array<double, 1U> result;
        result.set(vec.data(), static_cast<int>(vec.rows()));
        return result;
    }

    static inline coder::array<double, 2U> EigenMat2MatlabArray(Eigen::MatrixXd& mat)
    {
        coder::array<double, 2U> result;
        result.set(mat.data(), static_cast<int>(mat.rows()), static_cast<int>(mat.cols()));
        return result;
    }

    void updateParameters(const hippo_chain::LeastSquaresConfig& config, uint32_t level) override
    {
        if (!level) return;
        controlPenalty = std::pow(10, config.penalty);
        A_eigen.bottomRows(SIZE).diagonal().fill(controlPenalty);
        ROS_INFO("Updated LSQ-Parameters");
    }


public:
    MatlabLSQSolver(const int size)
    : LeastSquaresSolver(size)
    , solver()
    , A()
    , b()
    , x()
    , x0()
    , A_eigen(controlPenalty*Eigen::MatrixXd::Identity(SIZE, SIZE))
    , b_eigen(Eigen::VectorXd::Zero(SIZE))
    {
        x0.set_size(SIZE);
        for (auto it=x0.begin(), end=x0.end(); it!=end; it++) *it = 0;

        A.set(A_eigen.data(), static_cast<int>(A_eigen.rows()), static_cast<int>(A_eigen.cols()));

        b.set(b_eigen.data(), static_cast<int>(b_eigen.rows()));
    }


    void solve(const Eigen::MatrixXd& A_,
               const Eigen::VectorXd& b_,
               Eigen::VectorXd& x_)
    {
        assert(A_.rows() == b_.rows());
        assert(A_.cols() == SIZE);
        assert(x_.rows() == SIZE);

        if (A_.rows()+SIZE == A_eigen.rows()) {
            A_eigen.topRows(A_.rows()) = A_;
            b_eigen.topRows(b_.rows()) = b_;
        } else {
            A_eigen.resize(A_.rows()+SIZE, SIZE);
            b_eigen.resize(b_.rows()+SIZE);
            A_eigen.topRows(A_.rows()) = A_;
            b_eigen.topRows(b_.rows()) = b_;
            A_eigen.bottomRows(SIZE) = controlPenalty*Eigen::MatrixXd::Identity(SIZE, SIZE);
            b_eigen.bottomRows(SIZE) = Eigen::VectorXd::Zero(SIZE);
            A.set(A_eigen.data(), static_cast<int>(A_eigen.rows()), static_cast<int>(A_eigen.cols()));
            b.set(b_eigen.data(), static_cast<int>(b_eigen.rows()));
        }

#ifndef NDEBUG
        const clock_t begin_time = clock();

        debugger.addEntry("Size", SIZE);
        debugger.addEntry("A", A_);
        debugger.addEntry("desired b", b_);
#endif  // NDEBUG

        x.set(x_.data(), SIZE);

        solver.solveLSQ(A, b, x0, x);

        // std::copy(x.begin(), x.end(), x0.begin());

#ifndef NDEBUG
        double time = double(clock() - begin_time) * (1000.0/CLOCKS_PER_SEC);
        auto bActual = A_*x_;
        auto error = bActual-b_;

        debugger.addEntry("x", x_);
        debugger.addEntry("actual b", bActual);
        debugger.addEntry("error", error);
        debugger.addEntry("error norm", error.norm());
        debugger.addEntry("Solve time (in ms)", time);
        debugger.publish();
#endif  // NDEBUG
    }
};


#endif  // MATLABLSQSOLVER_H