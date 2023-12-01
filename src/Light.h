#ifndef LIGHT_H
#define LIGHT_H
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Ray.h"
#include "Geometry.h"
#include "Output.h"
using namespace std;

class Light {
    public:
        // Constructors
        Light();
        Light(const Light &l);

        // Setters
        void setType(string type) { this->type = type; }
        void setId(Eigen::Vector3f id) { this->id = id; }
        void setIs(Eigen::Vector3f is) { this->is = is; }
        void setTransform(Eigen::Matrix4f transform) { this->transform = transform; }
        void setUse(bool use) { this->use = use; }

        // Getters
        const string* getType() { return &type; }
        const Eigen::Vector3f* getId() { return &id; }
        const Eigen::Vector3f* getIs() { return &is; }
        const Eigen::Matrix4f* getTransform() { return &transform; }
        const bool* getUse() { return &use; }

        // Local illumination (Phong BRDF) method
        Eigen::Vector3f phong(vector<Geometry*> geometries, Output* output, vector<Light*> lights, Ray ray, Eigen::Vector3f pixel, Eigen::Vector3f right);
        Eigen::Vector3f phongContribution(vector<Geometry*> geometries, Geometry* closestGeo, Output* output, Light* light, Eigen::Vector3f lightPoint, Eigen::Vector3f surfaceNormal, Ray ray);

        // Global illumination via path tracing
        Geometry* closestGeometry(Ray ray, vector<Geometry*> geometries);
        Eigen::Vector3f globalillum(vector<Geometry*> geometries, Output* output, Light* light, Ray ray, int numbounces, int maxbounces, Eigen::Vector3f bgColor);
        Eigen::Vector3f hemisphereSample(Eigen::Vector3f surfaceNormal);

    private: 
        // Mandatory members
        string type;
        Eigen::Vector3f id, is;

        // Optional members
        Eigen::Matrix4f transform;
        Eigen::Vector3f normal;
        bool use;
};

class Point : public Light {   
    public:
        // Constructors
        Point();
        Point(const Point &p);
        Point(Eigen::Vector3f centre);

        // Setters
        void setCentre(Eigen::Vector3f centre) { this->centre = centre; }

        // Getters
        Eigen::Vector3f getCentre() { return centre; }

    private:
        // Mandatory point-light-specific members
        Eigen::Vector3f centre;
};

class Area : public Light {
    public:
        // Constructors
        Area();
        Area(const Area &a);
        Area(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, Eigen::Vector3f p4, unsigned int samples);

        // Setters
        void setP1(Eigen::Vector3f p1) { this->p1 = p1; }
        void setP2(Eigen::Vector3f p2) { this->p2 = p2; }
        void setP3(Eigen::Vector3f p3) { this->p3 = p3; }
        void setP4(Eigen::Vector3f p4) { this->p4 = p4; }
        void setNormal(Eigen::Vector3f normal) { this->normal = normal; }
        void setSamples(unsigned int samples) { this->samples = samples; }
        void setUsecenter(bool usecenter) { this->usecenter = usecenter; }

        // Getters
        const Eigen::Vector3f* getP1() { return &p1; }
        const Eigen::Vector3f* getP2() { return &p2; }
        const Eigen::Vector3f* getP3() { return &p3; }
        const Eigen::Vector3f* getP4() { return &p4; }
        const Eigen::Vector3f* getNormal() { return &normal; }
        const unsigned int* getSamples() { return &samples; }
        const bool* getUsecenter() { return &usecenter; }
    
    private:
        // Mandatory area-light-specific members
        Eigen::Vector3f p1, p2, p3, p4, normal;
        unsigned int samples;
        bool usecenter;
};
#endif