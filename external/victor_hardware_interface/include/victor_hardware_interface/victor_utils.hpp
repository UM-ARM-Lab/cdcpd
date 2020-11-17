#ifndef VICTOR_UTILS_HPP
#define VICTOR_UTILS_HPP

/**
 *   Useful utility function when working with Victor
 */


#include <victor_hardware_interface/JointValueQuantity.h>

namespace victor_utils
{
    static victor_hardware_interface::JointValueQuantity vectorToJvq(const std::vector<double> &v)
    {
        assert(v.size() == 7);
        
        victor_hardware_interface::JointValueQuantity jvq;
        jvq = victor_hardware_interface::JointValueQuantity();
        jvq.joint_1 = v[0];
        jvq.joint_2 = v[1];
        jvq.joint_3 = v[2];
        jvq.joint_4 = v[3];
        jvq.joint_5 = v[4];
        jvq.joint_6 = v[5];
        jvq.joint_7 = v[6];
        return jvq;
    };

    static std::vector<double> jvqToVector(const victor_hardware_interface::JointValueQuantity &jvq)
    {
        std::vector<double> v(7);
        v[0] = jvq.joint_1;
        v[1] = jvq.joint_2;
        v[2] = jvq.joint_3;
        v[3] = jvq.joint_4;
        v[4] = jvq.joint_5;
        v[5] = jvq.joint_6;
        v[6] = jvq.joint_7;
        return v;
    };

}



#endif
