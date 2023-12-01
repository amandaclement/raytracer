#ifndef GEOMETRY_H
#define GEOMETRY_H
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "Ray.h"
using namespace std;

class Geometry {
    public:
        // Constructors        
        Geometry();
        Geometry(const Geometry &g);

        // Setters
        void setType(string type) { this->type = type; }
        void setAc(Eigen::Vector3f ac) { this->ac = ac; }
        void setDc(Eigen::Vector3f dc) { this->dc = dc; }
        void setSc(Eigen::Vector3f sc) { this->sc = sc; }
        void setKa(float ka) { this->ka = ka; }
        void setKd(float kd) { this->kd = kd; }
        void setKs(float ks) { this->ks = ks; }
        void setPc(float pc) { this->pc = pc; }
        void setT(float t) { this->t = t; }
        void setVisible(bool visible) { this->visible = visible; }

        // Getters
        const string* getType() { return &type; }
        const Eigen::Vector3f* getAc() { return &ac; }
        const Eigen::Vector3f* getDc() { return &dc; }
        const Eigen::Vector3f* getSc() { return &sc; }
        const float* getKa() { return &ka; }
        const float* getKd() { return &kd; }
        const float* getKs() { return &ks; }
        const float* getPc() { return &pc; }
        const float* getT() { return &t; }
        const bool* getVisible() { return &visible; }

        // Abstract method, overloaded in derived classes
        virtual bool intersects(Ray ray) = 0;

    private: 
        // Mandatory Members
        string type;
        float ka, kd, ks, pc, t;
        Eigen::Vector3f ac, dc, sc;

        // Optional members
        bool visible;
};

class Sphere : public Geometry {
    public:
        // Constructors
        Sphere();
        Sphere(const Sphere &s);
        Sphere(Eigen::Vector3f centre, float radius);

        // Setters
        void setRadius(float radius) { this->radius = radius; }
        void setCentre(Eigen::Vector3f centre) { this->centre = centre; }

        // Getters
        const float* getRadius() { return &radius; }
        const Eigen::Vector3f* getCentre() { return &centre; }

        // Sphere's intersects method for checking if a ray intersects the sphere
        bool intersects(Ray ray);

    private: 
        // Mandatory sphere-specific members
        float radius; 
        Eigen::Vector3f centre;
};

class Rectangle : public Geometry {
    public:
        // Constructors
        Rectangle();
        Rectangle(const Rectangle &r);
        Rectangle(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, Eigen::Vector3f p4);

        // Setters
        void setP1(Eigen::Vector3f p1) { this->p1 = p1; }
        void setP2(Eigen::Vector3f p2) { this->p2 = p2; }
        void setP3(Eigen::Vector3f p3) { this->p3 = p3; }
        void setP4(Eigen::Vector3f p4) { this->p4 = p4; }
        void setNormal(Eigen::Vector3f normal) { this->normal = normal; }

        // Getters
        const Eigen::Vector3f* getP1() { return &p1; }
        const Eigen::Vector3f* getP2() { return &p2; }
        const Eigen::Vector3f* getP3() { return &p3; }
        const Eigen::Vector3f* getP4() { return &p4; }
        const Eigen::Vector3f* getNormal() { return &normal; }

        // Rectangles's intersects method for checking if a ray intersects the plane
        bool intersects(Ray ray);

    private:
        // Mandatory rectangle-specific members
        Eigen::Vector3f p1, p2, p3, p4, normal;
};
#endif