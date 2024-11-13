#ifndef GEOLOCATION_H
#define GEOLOCATION_H

#include <cmath>
#include <utility>

struct Coordinate {
    double lat, lon;
};

class Geolocation{
public:
    static constexpr double earthRadius = 6371000.0;

    static double Geolocation::haversine(const Coordinate& current_coordinate, const Coordinate& target_coordinate);
    static std::pair<double, double> Geolocation::haversineXY(const Coordinate& current_coordinate, const Coordinate& target_coordin);
    static double bearingAngle(const Coordinate& current_coordinate, const Coordinate& target_coordinate);
    static Coordinate lookAhead(const Coordinate& target_coordinate, double bearing_angle, double distance);
    static bool isinFieldOfView(const Coordinate& current_coordinate, const Coordinate& target_coordinate, 
                                double target_heading, double target_altitude);

private:
    static double degreesToRadians(double degrees);
    static double radiansToDegrees(double radians);
}; 


#endif