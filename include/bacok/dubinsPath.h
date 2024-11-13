#ifndef DUBINS_PATH_H
#define DUBINS_PATH_H

#include <vector>
#include <iostream>
#include <cmath>
#include <limits>

namespace dubins_path_planner {

enum PathType { LSL, LSR, RSL, RSR, RLR, LRL };

struct DubinsPath {
    double length;
    PathType type;
};

class DubinsPathCalculator {
public:
    DubinsPathCalculator(double turn_radius);
    DubinsPath computeDubinsPath(double x0, double y0, double theta0,
                                 double x1, double y1, double theta1);
private:
    double turn_radius;
    void dubinsLSL(double alpha, double beta, double d, DubinsPath& path);
    void dubinsRSR(double alpha, double beta, double d, DubinsPath& path);
    void dubinsLSR(double alpha, double beta, double d, DubinsPath& path);
    void dubinsRSL(double alpha, double beta, double d, DubinsPath& path);
    void dubinsRLR(double alpha, double beta, double d, DubinsPath& path);
    void dubinsLRL(double alpha, double beta, double d, DubinsPath& path);
};

} 

#endif
