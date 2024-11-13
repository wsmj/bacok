#ifndef QUATERNION_CONVERSION_H
#define QUATERNION_CONVERSION_H

#include <cmath>

class QuaternionConversion {
public:
    struct Quaternion {
        double w, x, y, z;
    };

    struct Euler {
        double roll, pitch, yaw;
    };

    static Euler toEuler(const Quaternion& q);
    static Quaternion toQuaternion(const Euler& e);
};

#endif 
