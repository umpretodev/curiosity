#include "curiosity_cpp_pkg/math_util.hpp"
#include <cmath>

double MathUtil::normalize_angle(double theta) {
    while (theta > M_PI) theta -= 2 * M_PI;
    while (theta < -M_PI) theta += 2 * M_PI;

    return theta;
};

double MathUtil::convert_graus_to_radians(double theta) {
    return theta * M_PI / 180.0;
}