#ifndef JOINTSTATEKALMANFILTER_H
#define JOINTSTATEKALMANFILTER_H


#include <ros/ros.h>
#include <Eigen/Dense>
#include <hippo_chain/include/common/DynamicReconfigureManager.h>
#include <hippo_chain/KalmanFilterConfig.h>


template<std::size_t Dim>
class KalmanFilter
{
private:
    typedef Eigen::Matrix<double,2*Dim,1> Vector;
    typedef Eigen::Matrix<double,2*Dim,2*Dim> Matrix;

    bool initialized;
    ros::Time lastPrediction;

    Vector belief;
    Matrix covariance;

    Matrix modelCov;
    Matrix measurementCov;
    Matrix stateTransitionMatrix;

    typedef boost::function<void(const hippo_chain::KalmanFilterConfig &, uint32_t)> CallbackType;
    CallbackType f;

    static inline std::unique_ptr<DynamicReconfigureManager<hippo_chain::KalmanFilterConfig>> dynamicReconfigureManager;


    void updateParameters(const hippo_chain::KalmanFilterConfig& config, uint32_t level)
    {
        if (level) initialized = false;

        for (uint32_t idx=0U; idx<Dim; idx++) {
            modelCov(idx, idx) = config.angleCovMod;
            modelCov(idx+Dim, idx+Dim) = config.omegaCovMod;
            measurementCov(idx, idx) = config.angleCovMeas;
            measurementCov(idx+Dim, idx+Dim) = config.omegaCovMeas;
        }
    }

    void predict()
    {
        const double dT = (ros::Time::now()-lastPrediction).toSec();
        lastPrediction = ros::Time::now();

        for (uint16_t idx=0U; idx<Dim; idx++) stateTransitionMatrix(idx, idx+Dim) = dT;

        belief = stateTransitionMatrix*belief;
        covariance = stateTransitionMatrix*covariance*stateTransitionMatrix.transpose() + modelCov;

        bound();
    }

    void bound()
    {
        double* it = belief.data();
        int idx = 0;
        for (; idx < Dim; idx++, it++) {
            if (*it > M_PI) {
                *it = M_PI;
                continue;
            }
            if (*it < -M_PI) {
                *it = -M_PI;
                continue;
            }
        }
    }


public:
    KalmanFilter()
    : initialized(false)
    , lastPrediction()
    , belief()
    , covariance()
    , modelCov(Matrix::Zero())
    , measurementCov(Matrix::Zero())
    , stateTransitionMatrix(Matrix::Identity())
    , f(boost::bind(&KalmanFilter::updateParameters, this, _1, _2))
    {
        if (!dynamicReconfigureManager) {
            std::string configFilePath = "";
            if (ros::param::get("dynamic_reconfigure_dir", configFilePath)) configFilePath += "KalmanFilterConfig.csv";
            dynamicReconfigureManager.reset(new DynamicReconfigureManager<hippo_chain::KalmanFilterConfig>("KalmanFilter", configFilePath));
        }
        dynamicReconfigureManager->registerCallback(f, this);
        dynamicReconfigureManager->initCallback(this);
    }

    ~KalmanFilter() = default;


    Vector update(const Eigen::Ref<const Vector>& z)
    {
        if (!initialized) {
            belief = z;
            covariance = measurementCov;
            initialized = true;
            lastPrediction = ros::Time::now();
            return z;
        }

        predict();

        const Vector innovation = z - belief;
        const Matrix kalmanGain = covariance * (covariance + measurementCov).inverse();

        covariance = (Matrix::Identity() - kalmanGain) * covariance;
        belief.noalias() += kalmanGain * innovation;

        bound();

        return belief;
    }
};


#endif  // JOINTSTATEKALMANFILTER_H