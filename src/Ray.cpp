#include "Ray.h"

// Ray default constructor
Ray::Ray() {
    this->centre = Eigen::Vector3f(0,0,0);
    this->dir = Eigen::Vector3f(0,0,0);
}

// Ray parameterized constructor
Ray::Ray(Eigen::Vector3f centre, Eigen::Vector3f dir) {
    this->centre = centre;
    this->dir = dir;
}