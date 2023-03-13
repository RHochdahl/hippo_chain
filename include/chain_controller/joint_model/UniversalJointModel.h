#ifndef UNIVERSALJOINTMODEL_H
#define UNIVERSALJOINTMODEL_H


#include "JointModel.h"
#include "bounds/BoundsEigen.h"


/**
 * @brief 2D-universal joint located on the x-axis of the robot
 * 
 */
class UniversalJointModel : public JointModel<2>
{
private:
    std::array<Axis, 2> axes;   // axes of rotation
    double jointPosX;           // x-coordinates of joint in child frame

    Eigen::Matrix3d R_2;        // rotation matrix of second rotation
    Eigen::Matrix3d R;          // rotation matrix of full rotation
    Eigen::Vector6d Phi_1;      // phi of first rotation before transformation


    void initA()
    {
        A = Eigen::Matrix6d::Identity();
        A(1, 5) = -(A(2, 4) = 2*jointPosX);
    }

    void calcA()
    {
        R_2 = calcR(axes[1], theta(1));
        R = R_2 * calcR(axes[0], theta(0));
        
        Eigen::Matrix3d temp = Eigen::Matrix3d::Zero();
        temp.row(1) -= R.row(2);
        temp.row(2) += R.row(1);
        temp.col(1) += R.col(2);
        temp.col(2) -= R.col(1);

        A.topLeftCorner(3, 3) = A.bottomRightCorner(3, 3) = R;
        A.topRightCorner(3, 3) = jointPosX * temp;
    }

    void initPhi()
    {
        Phi_1 = calcPhiVector(axes[0]);
        Phi.col(1) = calcPhiVector(axes[1]);
    }

    void calcPhi()
    {
        Phi.col(0).topRows<3>() = R_2 * Phi_1.topRows<3>();
        Phi.col(0).bottomRows<3>() = R_2 * Phi_1.bottomRows<3>();
    }

    void initTheta()
    {
        Theta.fill(0);
    }

    void calcTheta()
    {
        Eigen::Matrix3d dR_2 = Eigen::Matrix3d::Zero();

        switch (axes[0])
        {
        case Axis::x:
            dR_2.col(1) = zeta(1)*R_2.col(2);
            dR_2.col(2) = -zeta(1)*R_2.col(1);
            break;

        case Axis::y:
            dR_2.col(2) = zeta(1)*R_2.col(0);
            dR_2.col(0) = -zeta(1)*R_2.col(2);
            break;

        case Axis::z:
            dR_2.col(0) = zeta(1)*R_2.col(1);
            dR_2.col(1) = -zeta(1)*R_2.col(0);
            break;

        default:
            throw std::runtime_error("Unknown Axis");
            break;
        }

        Theta.col(1).topRows<3>() = dR_2 * Phi_1.topRows<3>();
        Theta.col(1).bottomRows<3>() = dR_2 * Phi_1.bottomRows<3>();
    }

    Eigen::Vector6d calcPhiVector(const Axis& axis) const
    {
        Eigen::Vector6d PhiVector;

        switch (axis)
        {
        case Axis::x:
            PhiVector << 0, 0, 0, 1, 0, 0;
            break;

        case Axis::y:
            PhiVector << 0, 0, jointPosX, 0, 1, 0;
            break;

        case Axis::z:
            PhiVector << 0, -jointPosX, 0, 0, 0, 1;
            break;

        default:
            throw std::runtime_error("Unknown Axis");
            break;
        }

        return PhiVector;
    }


public:
    static constexpr const char* jointTypeName = "universal";


    UniversalJointModel(const std::shared_ptr<ConfigProvider>& configProvider)
    {
        std::vector<int> axesInt;
        std::vector<double> poseLim, twistLim;
        if (!(configProvider->getValue("joint/x_pos", jointPosX) &&
              configProvider->getValue("joint/axes", axesInt) &&
              configProvider->getValue("joint/bounds/pose", poseLim) &&
              configProvider->getValue("joint/bounds/twist", twistLim)))
            ROS_FATAL("Could not retrieve joint parameters!");

        if (axesInt.size() != 2 || poseLim.size() != 2 || twistLim.size() != 2)
            ROS_FATAL("Array has unexpected size!");

        axes[0] = static_cast<Axis>(axesInt[0]);
        axes[1] = static_cast<Axis>(axesInt[1]);
        bounds.reset(new BoundsEigen<2>(poseLim, twistLim));

        initA();
        initPhi();
        initTheta();
    }

    ~UniversalJointModel() = default;
};


#endif  // UNIVERSALJOINTMODEL_H