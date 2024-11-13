#include "bacok/frameTransport.h"

FrameTransport::Euler FrameTransport::toEuler(const Quaternion& q) {
    Euler euler;

    euler.roll = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
    euler.pitch = asin(2 * (q.w * q.y - q.z * q.x));
    euler.yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

    return euler;
}

FrameTransport::Quaternion FrameTransport::toQuaternion(const Euler& e) {
    Quaternion q;

    double cy = cos(e.yaw * 0.5);
    double sy = sin(e.yaw * 0.5);
    double cp = cos(e.pitch * 0.5);
    double sp = sin(e.pitch * 0.5);
    double cr = cos(e.roll * 0.5);
    double sr = sin(e.roll * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}
