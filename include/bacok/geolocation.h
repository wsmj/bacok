#ifndef GEOLOCATION_H
#define GEOLOCATION_H

#include <cmath>
#include <iostream>

namespace geo_utils {

struct Coordinate {
    double lat, lon;
};

struct Position {
    // The X Y value describes the position relative to origin
    double x, y, dist;
};

class Geolocation{
public:
    static constexpr double earthRadius = 6371000.0;

    Position haversine(const Coordinate& current_coordinate, const Coordinate& target_coordinate);
    double bearingAngle(const Coordinate& current_coordinate, const Coordinate& target_coordinate);
    Coordinate lookAhead(const Coordinate& target_coordinate, double bearing_angle, double distance);
    bool isinFieldOfView(const Coordinate& current_coordinate, const Coordinate& target_coordinate, 
                                double target_heading, double target_altitude);

private:
    double degreesToRadians(double degrees);
    double radiansToDegrees(double radians);
}; 

}

#endif