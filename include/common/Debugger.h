#ifndef DEBUGGER_H
#define DEBUGGER_H


#include <ros/ros.h>
#include <Eigen/Dense>
#include <hippo_chain/DebugMsg.h>
#include <iostream>


#ifndef NDEBUG
class Debugger
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    hippo_chain::DebugMsg msg;
public:
    Debugger(const std::string& ns, const std::string& topic)
    : nh("/debug" + ns)
    , pub(nh.advertise<hippo_chain::DebugMsg>(topic, 1))
    , msg()
    {}

    ~Debugger()
    {
        nh.shutdown();
    }


    template<typename T>
    void addEntry(const std::string& name, const T& obj)
    {
        addEntry(name, obj, std::conditional_t<std::is_arithmetic_v<T>, std::true_type, std::false_type>{});
    }

    template<typename T>
    void addEntry(const std::string& name, const T& obj, std::false_type)
    {
        Eigen::Matrix<double, obj.RowsAtCompileTime, obj.ColsAtCompileTime> mat = obj;
        addEntry(name, mat);
    }

    template<typename Scalar, int Rows, int Cols>
    void addEntry(const std::string& name, const Eigen::Matrix<Scalar, Rows, Cols>& mat)
    {
        if constexpr (Rows == Eigen::Dynamic || Cols == Eigen::Dynamic) {
            if (mat.rows() == 1 || mat.cols() == 1) {
                addEntry(name, mat.data(), mat.size());
                return;
            }
        } else if constexpr (Cols == 1 || Rows == 1) {
            addEntry(name, mat.data(), mat.SizeAtCompileTime);
            return;
        }

        std::ostringstream out;
        out << "[";
        for (int i=0; i<mat.rows(); i++) {
            out << "[";
            for (int j=0; j<mat.cols(); j++) {
                out << mat(i, j) << ", ";
            }
            out.seekp(-2,out.cur);
            out << "]; ";
        }
        out.seekp(-2,out.cur);
        out << "]";
        addEntry(name, out.str());
    }

    template<typename T>
    void addEntry(const std::string& name, const T& data, std::true_type)
    {
        addEntry(name, std::to_string(data));
    }

    template<typename T>
    void addEntry(const std::string& name, const T* begin, const std::size_t size)
    {
        if (size == 0) return;
        const T* const end = begin + size;
        std::ostringstream out;
        out << "[" << *(begin++);
        for (; begin<end; begin++) {
            out << ", " << *begin;
        }
        out << "]";
        addEntry(name, out.str());
    }

    void addEntry(const std::string& name, const std::string& data)
    {
        msg.data.push_back(name + ": " + data);
    }

    void publish()
    {
        if (pub.getNumSubscribers()) {
            msg.header.stamp = ros::Time::now();
            pub.publish(msg);
        }
        msg.data.clear();
    }
};

#else
class Debugger
{
public:
    Debugger(const std::string&, const std::string&) {}
    template<typename ... T> void addEntry(T...) {}
    void publish() {}
};
#endif  // NDEBUG

#endif  // DEBUGGER_H