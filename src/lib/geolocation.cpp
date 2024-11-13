#include <bacok/geolocation.h>

double Geolocation::degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

double Geolocation::radiansToDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

double Geolocation::haversine(const Coordinate& current_coordinate, const Coordinate& target_coordinate) {
    double t_lat = radiansToDegrees(target_coordinate.lat);
    double t_lon = radiansToDegrees(target_coordinate.lon);
    double c_lat = radiansToDegrees(current_coordinate.lat);
    double c_lon = radiansToDegrees(current_coordinate.lon);
    double d_lat = t_lat - c_lat;
    double d_lon = t_lon - c_lon;

    double a = std::sin(d_lat / 2) * std::sin(d_lat / 2) +
               std::cos(c_lat) * std::cos(t_lat) * std::sin(d_lon / 2) * std::sin(d_lon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return earthRadius * c;
}

std::pair<double, double> Geolocation::haversineXY(const Coordinate& target, const Coordinate& current) {
    double t_lat = radiansToDegrees(target.lat);
    double t_lon = radiansToDegrees(target.lon);
    double c_lat = radiansToDegrees(current.lat);
    double c_lon = radiansToDegrees(current.lon);

    double d_lat = t_lat - c_lat;
    double y = earthRadius * d_lat;

    double d_lon = t_lon - c_lon;
    double x = earthRadius * d_lon * std::cos(c_lat);

    return { x, y };
}

double Geolocation::bearingAngle(const Coordinate& current_coordinate, const Coordinate& target_coordinate) {
    double t_lat = radiansToDegrees(target_coordinate.lat);
    double t_lon = radiansToDegrees(target_coordinate.lon);
    double c_lat = radiansToDegrees(current_coordinate.lat);
    double c_lon = radiansToDegrees(current_coordinate.lon);
    double d_lon = t_lon - c_lon;

    double a = std::sin(d_lon) * std::cos(t_lat);
    double b = std::cos(c_lat) * std::sin(t_lat) - std::sin(c_lat) * std::cos(t_lat) * std::cos(d_lon);

    double angle = radiansToDegrees(std::atan2(a, b));
    return std::fmod(angle + 360.0, 360.0);
}

Coordinate Geolocation::lookAhead(const Coordinate& target_coordinate, double bearing_angle, double distance) {
    double t_lat = degreesToRadians(target_coordinate.lat);
    double t_lon = degreesToRadians(target_coordinate.lon);
    double bearing = degreesToRadians(bearing_angle);

    double p_lat = std::asin(std::sin(t_lat) * std::cos(distance / earthRadius) +
                             std::cos(t_lat) * std::sin(distance / earthRadius) * std::cos(bearing));
    double p_lon = t_lon + std::atan2(std::sin(bearing) * std::sin(distance / earthRadius) * std::cos(t_lat),
                                      std::cos(distance / earthRadius) - std::sin(t_lat) * std::sin(p_lat));

    return { radiansToDegrees(p_lat), radiansToDegrees(p_lon) };
}

bool Geolocation::isinFieldOfView(const Coordinate& current_coordinate, const Coordinate& target_coordinate, double target_heading, double target_altitude) {
    int fovRange = 60;
    int fovDistance_meter = 20;
    
    double angleDiff = bearingAngle(current_coordinate, target_coordinate);
    double lowerBound = std::fmod(target_heading - fovRange / 2.0 + 360.0, 360.0);
    double upperBound = std::fmod(target_heading + fovRange / 2.0, 360.0);

    if ((angleDiff >= lowerBound && angleDiff <= upperBound) || 
        (lowerBound > upperBound && (angleDiff >= lowerBound || angleDiff <= upperBound))) {
        return haversine(target_coordinate, current_coordinate) <= fovDistance_meter;
    }
    return false;
}