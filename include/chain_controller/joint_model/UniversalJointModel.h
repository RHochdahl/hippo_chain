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
        Phi.col(0) = calcPhiVector(axes[0]);
        Phi.col(1) = calcPhiVector(axes[1]);
    }

    void calcPhi()
    {
        Phi.col(0) = calcPhiVector(axes[0],R_2);
    }

    void initTheta()
    {
        Theta.fill(0);
    }

    void calcTheta()
    {
        Eigen::Matrix3d dR_2 = Eigen::Matrix3d::Zero();

        switch (axes[1])
        {
        case Axis::x:
            dR_2.col(1) = R_2.col(2);
            dR_2.col(2) = -R_2.col(1);
            break;

        case Axis::y:
            dR_2.col(2) = R_2.col(0);
            dR_2.col(0) = -R_2.col(2);
            break;

        case Axis::z:
            dR_2.col(0) = R_2.col(1);
            dR_2.col(1) = -R_2.col(0);
            break;

        default:
            throw std::runtime_error("Unknown Axis");
            break;
        }

        Theta.col(0) = zeta(1)*calcPhiVector(axes[0], dR_2);
    }

    Eigen::Vector6d calcPhiVector(const Axis& axis) const
    {
        return calcPhiVector(axis, Eigen::Matrix3d::Identity());
    }

    Eigen::Vector6d calcPhiVector(const Axis& axis, const Eigen::Matrix3d& Ri) const
    {
        int col;

        switch (axis)
        {
        case Axis::x:
            col = 0;
            break;

        case Axis::y:
            col = 1;
            break;

        case Axis::z:
            col = 2;
            break;

        default:
            throw std::runtime_error("Unknown Axis");
            break;
        }
        Eigen::Vector6d PhiVector;

        PhiVector(0) = 0;
        PhiVector(1) = -jointPosX * Ri(2,col);
        PhiVector(2) = jointPosX * Ri(1,col);
        PhiVector.bottomRows(3) = Ri.col(col);

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