#ifndef RAY_H
#define RAY_H
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std;

class Ray {
    public:

        // Setters
        void setCentre(Eigen::Vector3f centre) { this->centre = centre; }
        void setDir(Eigen::Vector3f dir) { this->dir = dir; }

        // Getters
        Eigen::Vector3f getCentre() { return centre; }
        Eigen::Vector3f getDir() { return dir; }

        // Constructors
        Ray();
        Ray(Eigen::Vector3f centre, Eigen::Vector3f dir);

    private:
        Eigen::Vector3f centre, dir;
};

#endif