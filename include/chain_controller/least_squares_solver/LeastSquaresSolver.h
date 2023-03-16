#ifndef LEASTSQUARESSOLVER_H
#define LEASTSQUARESSOLVER_H


#include <Eigen/Dense>
#include <ctime>
#include <hippo_chain/include/common/Debugger.h>

#include <hippo_chain/include/common/DynamicReconfigureManager.h>
#include <hippo_chain/LeastSquaresConfig.h>


class LeastSquaresSolver
{
protected:
    Debugger debugger;
    double controlPenalty;

    typedef boost::function<void(const hippo_chain::LeastSquaresConfig &, uint32_t)> CallbackType;
    CallbackType f;

    static inline std::unique_ptr<DynamicReconfigureManager<hippo_chain::LeastSquaresConfig>> dynamicReconfigureManager;


    virtual void updateParameters(const hippo_chain::LeastSquaresConfig& config, uint32_t level)
    {
        if (!level) return;
        controlPenalty = std::pow(10, config.penalty);
        ROS_INFO("Updated LSQ-Parameters");
    }


public:
    const int SIZE;

    LeastSquaresSolver(const int size)
    : debugger("", "lsq_solver")
    , SIZE(size)
    , controlPenalty()
    , f(boost::bind(&LeastSquaresSolver::updateParameters, this, _1, _2))
    {
        if (!dynamicReconfigureManager) {
            std::string configFilePath = "";
            if (ros::param::get("dynamic_reconfigure_dir", configFilePath)) configFilePath += "LSQConfig.csv";
            dynamicReconfigureManager.reset(new DynamicReconfigureManager<hippo_chain::LeastSquaresConfig>("LeastSquaresSolver", configFilePath));
        }
        dynamicReconfigureManager->registerCallback(f, this);
        dynamicReconfigureManager->initCallback(this);
    }

    ~LeastSquaresSolver()
    {
        dynamicReconfigureManager->removeCallback(this);
    }

    virtual void solve(const Eigen::MatrixXd& B,
                       const Eigen::VectorXd& eta,
                       Eigen::VectorXd& nu) = 0;
};


#endif  // LEASTSQUARESSOLVER_H