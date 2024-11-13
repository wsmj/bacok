#include <bacok/dubinsPath.h>

#define PI M_PI

namespace dubins_path_planner {

DubinsPathCalculator::DubinsPathCalculator(double turn_radius)
    : turn_radius(turn_radius) {}

void DubinsPathCalculator::dubinsLSL(double alpha, double beta, double d, DubinsPath& path) {
    double tmp = d + sin(alpha) - sin(beta);
    double p_squared = 2 + d * d - 2 * cos(alpha - beta) + 2 * d * (sin(alpha) - sin(beta));
    if (p_squared >= 0) {
        double p = sqrt(p_squared);
        double t = fmod(-alpha + atan2(cos(beta) - cos(alpha), tmp), 2 * PI);
        double q = fmod(beta - atan2(cos(beta) - cos(alpha), tmp), 2 * PI);
        path.length = t + p + q;
        path.type = LSL;
    } else {
        path.length = std::numeric_limits<double>::infinity();
    }
}

void DubinsPathCalculator::dubinsRSR(double alpha, double beta, double d, DubinsPath& path) {
    double tmp = d - sin(alpha) + sin(beta);
    double p_squared = 2 + d * d - 2 * cos(alpha - beta) - 2 * d * (sin(alpha) - sin(beta));
    if (p_squared >= 0) {
        double p = sqrt(p_squared);
        double t = fmod(alpha - atan2(cos(alpha) - cos(beta), tmp), 2 * PI);
        double q = fmod(-beta + atan2(cos(alpha) - cos(beta), tmp), 2 * PI);
        path.length = t + p + q;
        path.type = RSR;
    } else {
        path.length = std::numeric_limits<double>::infinity();
    }
}

void DubinsPathCalculator::dubinsLSR(double alpha, double beta, double d, DubinsPath& path) {
    double p_squared = -2 + d * d + 2 * cos(alpha - beta) + 2 * d * (sin(alpha) + sin(beta));
    if (p_squared >= 0) {
        double p = sqrt(p_squared);
        double tmp = atan2(-cos(alpha) - cos(beta), d + sin(alpha) + sin(beta)) - alpha;
        double t = fmod(tmp, 2 * PI);
        double q = fmod(beta - atan2(-cos(alpha) - cos(beta), d + sin(alpha) + sin(beta)), 2 * PI);
        path.length = t + p + q;
        path.type = LSR;
    } else {
        path.length = std::numeric_limits<double>::infinity();
    }
}

void DubinsPathCalculator::dubinsRSL(double alpha, double beta, double d, DubinsPath& path) {
    double p_squared = -2 + d * d + 2 * cos(alpha - beta) - 2 * d * (sin(alpha) + sin(beta));
    if (p_squared >= 0) {
        double p = sqrt(p_squared);
        double tmp = atan2(cos(alpha) + cos(beta), d - sin(alpha) - sin(beta)) + alpha;
        double t = fmod(tmp, 2 * PI);
        double q = fmod(beta - tmp, 2 * PI);
        path.length = t + p + q;
        path.type = RSL;
    } else {
        path.length = std::numeric_limits<double>::infinity();
    }
}

void DubinsPathCalculator::dubinsRLR(double alpha, double beta, double d, DubinsPath& path) {
    double tmp = (6 - d * d + 2 * cos(alpha - beta) + 2 * d * (sin(alpha) - sin(beta))) / 8;
    if (abs(tmp) <= 1) {
        double p = acos(tmp);
        double t = fmod(alpha - atan2(cos(alpha) - cos(beta), d - sin(alpha) + sin(beta)) + p / 2, 2 * PI);
        double q = fmod(p, 2 * PI);
        path.length = t + q + t;
        path.type = RLR;
    } else {
        path.length = std::numeric_limits<double>::infinity();
    }
}

void DubinsPathCalculator::dubinsLRL(double alpha, double beta, double d, DubinsPath& path) {
    double tmp = (6 - d * d + 2 * cos(alpha - beta) - 2 * d * (sin(alpha) - sin(beta))) / 8;
    if (abs(tmp) <= 1) {
        double p = acos(tmp);
        double t = fmod(-alpha + atan2(cos(beta) - cos(alpha), d + sin(alpha) - sin(beta)) + p / 2, 2 * PI);
        double q = fmod(p, 2 * PI);
        path.length = t + q + t;
        path.type = LRL;
    } else {
        path.length = std::numeric_limits<double>::infinity();
    }
}

DubinsPath DubinsPathCalculator::computeDubinsPath(double x0, double y0, double theta0,
                                                   double x1, double y1, double theta1) {
    double dx = x1 - x0;
    double dy = y1 - y0;
    double D = sqrt(dx * dx + dy * dy);
    double d = D / turn_radius;
    double theta = atan2(dy, dx);
    double alpha = fmod(theta0 - theta + 2 * PI, 2 * PI);
    double beta = fmod(theta1 - theta + 2 * PI, 2 * PI);

    DubinsPath paths[6];
    dubinsLSL(alpha, beta, d, paths[LSL]);
    dubinsRSR(alpha, beta, d, paths[RSR]);
    dubinsLSR(alpha, beta, d, paths[LSR]);
    dubinsRSL(alpha, beta, d, paths[RSL]);
    dubinsRLR(alpha, beta, d, paths[RLR]);
    dubinsLRL(alpha, beta, d, paths[LRL]);

    double min_length = std::numeric_limits<double>::infinity();
    DubinsPath best_path;
    for (int i = 0; i < 6; ++i) {
        if (paths[i].length < min_length) {
            min_length = paths[i].length;
            best_path = paths[i];
        }
    }

    return best_path;
}

} 