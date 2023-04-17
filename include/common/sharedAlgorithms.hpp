#ifndef SHAREDALGORITHMS_H
#define SHAREDALGORITHMS_H


#include <ros/ros.h>
#include "typedefs.h"
#include "string.h"


namespace shared
{
    template <typename T>
    static inline int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    static inline bool isUnitQuaternion(const Eigen::Vector4d& quat)
    {
        return (std::abs(quat.squaredNorm() - 1.0) < 1e-3);
    }

    static inline bool isUnitQuaternion(const Eigen::Quaterniond& quat)
    {
        return (std::abs(quat.squaredNorm() - 1.0) < 1e-3);
    }

    static inline Eigen::Vector6d cross6(const Eigen::Vector6d& first, const Eigen::Vector6d& second)
    {
        Eigen::Vector6d res;
        res.topRows<3>() = first.bottomRows<3>().cross(second.topRows<3>()) + first.topRows<3>().cross(second.bottomRows<3>());
        res.bottomRows<3>() = first.bottomRows<3>().cross(second.bottomRows<3>());
        return res;
    }

    static inline double sat(const double in, const double rho)
    {
        const double out = rho*in;
        if (out > 1.0) return 1.0;
        if (out < -1.0) return -1.0;
        return out;
    }

    template<int Dim>
    static inline Eigen::Matrix<double,Dim,1> sat(const Eigen::Matrix<double,Dim,1>& in, const double rho)
    {
        Eigen::Matrix<double,Dim,1> out;
        for (auto inIt=in.data(), outIt=out.data(), end=outIt+Dim; outIt!=end; inIt++, outIt++) {
            *outIt = sat(*inIt, rho);
        }
        return out;
    }

    template<int Dim>
    static inline Eigen::Matrix<double,Dim,1> sat(const Eigen::Matrix<double,Dim,1>& in, const Eigen::Matrix<double,Dim,1>& rho)
    {
        Eigen::Matrix<double,Dim,1> out;
        for (auto inIt=in.data(), rhoIt=rho.data(), outIt=out.data(), end=outIt+Dim; outIt!=end; inIt++, rhoIt++, outIt++) {
            *outIt = sat(*inIt, *rhoIt);
        }
        return out;
    }


    template<std::size_t N>
    static inline boost::array<double, N> toArray(const double* data, const std::size_t size)
    {
        boost::array<double, N> ret;
        std::copy(data, data+size, ret.data());
        std::fill_n(ret.data()+size, N-size, 0.0);
        return ret;
    }

    template<std::size_t N>
    static inline boost::array<double, N> toArray(const double val)
    {
        return boost::array<double, N>({val});
    }

    template<std::size_t N, int M>
    static inline boost::array<double, N> toArray(const Eigen::Matrix<double,M,1>& vec)
    {
        return toArray<N>(vec.data(), M);
    }

    template<std::size_t N>
    static inline boost::array<double, N> toArray(const Eigen::VectorXd& vec)
    {
        return toArray<N>(vec.data(), vec.rows());
    }


    static inline int getID(const std::string& name)
    {
        try
        {
            return std::stoi(name.substr(name.find_last_not_of("0123456789") + 1));
        }
        catch(const std::invalid_argument& e)
        {
            ROS_ERROR("Could not read id from name '%s'!", name.c_str());
            return std::numeric_limits<int>::min();
        }
    }
}

#endif  // SHAREDALGORITHMS_H