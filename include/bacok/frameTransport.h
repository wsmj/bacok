#ifndef QUATERNION_CONVERSION_H
#define QUATERNION_CONVERSION_H

#include <cmath>

class FrameTransport {
public:
    struct Quaternion {
        float w, x, y, z;
    };

    struct Euler {
        float roll, pitch, yaw;
    };

    static Euler toEuler(const Quaternion& q);
    static Quaternion toQuaternion(const Euler& e);
};

#endif 
