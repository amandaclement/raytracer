#ifndef RAYTRACER_H
#define RAYTRACER_H
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Ray.h"
#include "Geometry.h"
#include "Light.h"
#include "Output.h"
#include "../external/json.hpp"
#include "simpleppm.h"
using namespace std;

class RayTracer {
    public:
        // Members
        Ray ray;
        vector<Geometry*> geometries;
        vector<Light*> lights;
        vector<Output*> outputs;

        // Constructors
        RayTracer(nlohmann::json j);

        // Methods for JSON parsing
        void parseGeometry(nlohmann::json& j);
        void parseLights(nlohmann::json& j);
        void parseOutput(nlohmann::json& j);

        // Methods for rendering image
        int render();
        void run();
};
#endif
