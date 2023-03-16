#ifndef MATLABLSQSOLVER2_H
#define MATLABLSQSOLVER2_H


#include <memory>
#include <algorithm>
#include <iostream>
#include "LeastSquaresSolver.h"

#include <Eigen/Dense>

#include <cstddef>
#include <cstdlib>

// Matlab
#include <hippo_chain/lib/matlab_lsq_solver/interior-point/LSQSolver2.h>
#include <hippo_chain/lib/matlab_lsq_solver/rt_nonfinite.h>
#include <hippo_chain/lib/matlab_lsq_solver/rtwtypes.h>
#include <hippo_chain/lib/matlab_lsq_solver/coder_array.h>



class MatlabLSQSolver2 : public LeastSquaresSolver
{
private:
    LSQSolver2 solver;
    coder::array<double, 2U> Q;
    coder::array<double, 1U> q;
    coder::array<double, 1U> x;
    coder::array<double, 1U> x0;

    Eigen::MatrixXd Q_eigen;
    Eigen::VectorXd q_eigen;


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


public:
    MatlabLSQSolver2(const int size)
    : LeastSquaresSolver(size)
    , solver()
    , Q()
    , q()
    , x()
    , x0()
    , Q_eigen(controlPenalty*Eigen::MatrixXd::Identity(SIZE, SIZE))
    , q_eigen(Eigen::VectorXd::Zero(SIZE))
    {
        x0.set_size(SIZE);
        for (auto it=x0.begin(), end=x0.end(); it!=end; it++) *it = 0;

        Q.set(Q_eigen.data(), static_cast<int>(Q_eigen.rows()), static_cast<int>(Q_eigen.cols()));

        q.set(q_eigen.data(), static_cast<int>(q_eigen.rows()));
    }


    void solve(const Eigen::MatrixXd& B,
               const Eigen::VectorXd& eta,
               Eigen::VectorXd& nu)
    {
        assert(B.rows() == eta.rows());
        assert(B.cols() == SIZE);
        assert(nu.rows() == SIZE);

        Q_eigen.noalias() = B.transpose()*B;
        Q_eigen.diagonal().array() += controlPenalty;
        q_eigen.noalias() = -B.transpose() * eta;


#ifndef NDEBUG
        const clock_t begin_time = clock();

        debugger.addEntry("Size", SIZE);
        debugger.addEntry("B", B);
        debugger.addEntry("desired eta", eta);
#endif  // NDEBUG

        x.set(nu.data(), SIZE);

        solver.solveLSQ2(Q, q, x0, x);

        // std::copy(x.begin(), x.end(), x0.begin());

#ifndef NDEBUG
        double time = double(clock() - begin_time) * (1000.0/CLOCKS_PER_SEC);
        auto etaActual = B*nu;
        auto error = etaActual-eta;

        debugger.addEntry("nu", nu);
        debugger.addEntry("actual eta", etaActual);
        debugger.addEntry("error", error);
        debugger.addEntry("error norm", error.norm());
        debugger.addEntry("Solve time (in ms)", time);
        debugger.publish();
#endif  // NDEBUG
    }
};


#endif  // MATLABLSQSOLVER2_H