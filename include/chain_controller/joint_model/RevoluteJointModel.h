#ifndef REVOLUTEJOINTMODEL_H
#define REVOLUTEJOINTMODEL_H


#include "JointModel1d.h"
#include <string>


/**
 * @brief 1D-revolute joint located on the x-axis of the robot
 * 
 */
class RevoluteJointModel : public JointModel1d
{
private:
    enum Axis
    {
        undefined = 0,
        x = 1,
        y = 2,
        z = 3
    } axis;                 // axis of rotation
    double jointPosX;       // x-coordinates of joint in child frame


    void initA()
    {
        A = Eigen::Matrix6d::Identity();
        A(1, 5) = -(A(2, 4) = 2*jointPosX);
    }

    void calcA()
    {
        const double sinTheta = std::sin(theta);
        const double cosTheta = std::cos(theta);

        switch (axis)
        {
        case Axis::x:
            /*
             * [[      1       0       0       0       0       0      ]
             *  [      0      c{}     s{}      0       0     -2*a     ]
             *  [      0     -s{}     c{}      0      2*a      0      ]
             *  [      0       0       0       1       0       0      ]
             *  [      0       0       0       0      c{}     s{}     ]
             *  [      0       0       0       0     -s{}     c{}     ]]
             */
            A(1, 1) = A(2, 2) = A(4, 4) = A(5, 5) = cosTheta;
            A(2, 1) = A(5, 4) = -(A(1, 2) = A(4, 5) = sinTheta);
            break;

        case Axis::y:
            /*
             * [[     c{}      0     -s{}      0    -a*s{}     0      ]
             *  [      0       1       0    -a*s{}     0   -a*(c{}+1) ]
             *  [     s{}      0      c{}      0    a*(c{}+1)  0      ]
             *  [      0       0       0      c{}      0     -s{}     ]
             *  [      0       0       0       0       1       0      ]
             *  [      0       0       0      s{}      0      c{}     ]]
             */
            A(0, 0) = A(2, 2) = A(3, 3) = A(5, 5) = cosTheta;
            A(0, 2) = A(3, 5) = -(A(2, 0) = A(5, 3) = sinTheta);
            A(0, 4) = A(1, 3) = -jointPosX * sinTheta;
            A(1, 5) = -(A(2, 4) = jointPosX * (cosTheta + 1));
            break;

        case Axis::z:
            /*
             * [[     c{}     s{}      0       0       0    -a*s{}    ]
             *  [    -s{}     c{}      0       0       0   -a*(c{}+1) ]
             *  [      0       0       1    -a*s{}  a*(c{}+1)  0      ]
             *  [      0       0       0      c{}     s{}      0      ]
             *  [      0       0       0     -s{}     c{}      0      ]
             *  [      0       0       0       0       0       1      ]]
             */
            A(0, 0) = A(1, 1) = A(3, 3) = A(4, 4) = cosTheta;
            A(1, 0) = A(4, 3) = -(A(0, 1) = A(3, 4) = sinTheta);
            A(0, 5) = A(2, 3) = -jointPosX * sinTheta;
            A(1, 5) = -(A(2, 4) = jointPosX * (cosTheta + 1));
            break;

        default:
            throw std::runtime_error("Unknown Axis");
            break;
        }
    }

    void calcPhi()
    {
        switch (axis)
        {
        case Axis::x:
            Phi << 0, 0, 0, 0, 1, 0;
            break;

        case Axis::y:
            Phi << 0, 0, jointPosX, 0, 1, 0;
            break;

        case Axis::z:
            Phi << 0, -jointPosX, 0, 0, 0, 1;
            break;

        default:
            throw std::runtime_error("Unknown Axis");
            break;
        }
    }

    void calcTheta()
    {
        Theta.fill(0);
    }

    void calcT()
    {
        T = 1.0;
    }


public:
    static constexpr const char* jointTypeName = "revolute";


    RevoluteJointModel(const std::shared_ptr<ConfigProvider>& configProvider)
    {
        int axisInt;
        double poseLim, twistLim;
        if (!(configProvider->getValue("joint/x_pos", jointPosX) &&
              configProvider->getValue("joint/axis", axisInt) &&
              configProvider->getValue("joint/bounds/pose", poseLim) &&
              configProvider->getValue("joint/bounds/twist", twistLim)))
            ROS_FATAL("Could not retrieve joint parameters!");
        axis = static_cast<Axis>(axisInt);
        bounds.reset(new BoundsDouble(poseLim, -poseLim, twistLim));

        initA();
        calcPhi();
        calcTheta();
        calcT();
    }

    ~RevoluteJointModel()
    {}
};


#endif  // REVOLUTEJOINTMODEL_H