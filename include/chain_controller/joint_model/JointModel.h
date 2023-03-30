#ifndef JOINTMODEL_H
#define JOINTMODEL_H


#include <hippo_chain/include/common/typedefs.h>
#include <hippo_chain/include/common/ConfigProvider.h>
#include <hippo_chain/include/chain_controller/state/StateProvider.h>
#include "bounds/Bounds.h"


template<std::size_t dof>
class JointModel
{
protected:
    enum Axis
    {
        undefined = 0,
        x = 1,
        y = 2,
        z = 3
    };

    virtual void calcA() = 0;
    virtual void calcPhi() = 0;
    virtual void calcTheta() = 0;


    static inline Eigen::Matrix3d calcR(const Axis& axis, const double angle)
    {
        Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();

        switch (axis)
        {
        case Axis::x:
            /*
             * [[      1       0       0       ]
             *  [      0      c{}     s{}      ]
             *  [      0     -s{}     c{}      ]]
             */
            mat(1, 1) = mat(2, 2) = std::cos(angle);
            mat(2, 1) = -(mat(1, 2) = std::sin(angle));
            break;

        case Axis::y:
            /*
             * [[     c{}      0     -s{}     ]
             *  [      0       1       0      ]
             *  [     s{}      0      c{}     ]]
             */
            mat(0, 0) = mat(2, 2) = std::cos(angle);
            mat(0, 2) = -(mat(2, 0) = std::sin(angle));
            break;

        case Axis::z:
            /*
             * [[     c{}     s{}      0      ]
             *  [    -s{}     c{}      0      ]
             *  [      0       0       1      ]]
             */
            mat(0, 0) = mat(1, 1) = std::cos(angle);
            mat(1, 0) = -(mat(0, 1) = std::sin(angle));
            break;

        default:
            throw std::runtime_error("Unknown Axis");
            break;
        }

        return mat;
    }


public:
    static constexpr int DOF = dof;
    typedef typename std::conditional<DOF == 1, double, Eigen::Matrix<double, DOF, 1>>::type JointVector;
    typedef typename std::conditional<DOF == 1, double, Eigen::Matrix<double, DOF, DOF>>::type JointMatrix;

    JointVector theta;                      // joint coordinates
    JointVector zeta;                       // joint velocities
    Eigen::Matrix6d A;                      // transformation matrix to parent frame
    Eigen::Matrix<double, 6, dof> Phi;      // jacobian of joint velocities to relative velocities
    Eigen::Matrix<double, 6, dof> Theta;    // time derivative of Phi
    Eigen::Vector6d xiRel;                  // relative velocity to parent (in child frame)

    std::unique_ptr<Bounds<JointVector>> bounds;


    void enforceBounds(JointVector& desPose, JointVector& desTwist) const
    {
        if (!bounds->enforceBounds(desPose, desTwist)) ROS_WARN_THROTTLE(5.0, "Joint target out of bounds!");
    }


    template<typename Derived>
    auto mapVelocity(const Eigen::MatrixBase<Derived>& velocity) const
    {
        return Phi * velocity;
    }

    template<typename Derived1,
             typename Derived2>
    auto mapAcceleration(const Eigen::MatrixBase<Derived1>& acceleration,
                         const Eigen::MatrixBase<Derived2>& velocity) const
    {
        return Phi * acceleration + Theta * velocity;
    }

    template<typename Derived>
    auto transform(const Eigen::MatrixBase<Derived>& input) const
    {
        return A * input;
    }

    template<typename Derived>
    auto transposedTransform(const Eigen::MatrixBase<Derived>& input) const
    {
        return A.transpose() * input;
    }


    void update(const std::shared_ptr<StateProvider> newState)
    {
        theta = newState->get_pose<JointVector>();
        zeta = newState->get_twist<JointVector>();
        calcA();
        calcPhi();
        calcTheta();
        xiRel = mapVelocity(zeta);
        if (!bounds->checkBounds(theta, zeta)) ROS_WARN_THROTTLE(5.0, "Joint out of bounds!");
    }
};


#endif  // JOINTMODEL_H