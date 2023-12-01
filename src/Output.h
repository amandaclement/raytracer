#ifndef OUTPUT_H
#define OUTPUT_H
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std;

class Output {
    public:
        // Constructors
        Output();
        Output(const Output &o);

        // Setters
        void setFilename(string filename) { this->filename = filename; }
        void setWidth(unsigned int width) { this->width = width; }
        void setHeight(unsigned int height) { this->height = height; }
        void setFov(float fov) { this->fov = fov; }
        void setUp(Eigen::Vector3f up) { this->up = up; }
        void setLookat(Eigen::Vector3f lookat) { this->lookat = lookat; }
        void setAi(Eigen::Vector3f ai) { this->ai = ai; }
        void setBkc(Eigen::Vector3f bkc) { this->bkc = bkc; }
        void setCentre(Eigen::Vector3f centre) { this->centre = centre; }
        void setMaxbounces(float maxbounces) { this->maxbounces = maxbounces; }
        void setProbterminate(float probterminate) { this->probterminate = probterminate; }
        void setRaysperpixel(Eigen::Vector3f raysperpixel) { this->raysperpixel = raysperpixel; }
        void setAntialiasing(bool antialiasing) { this->antialiasing = antialiasing; }
        void setTwosiderender(bool twosiderender) { this->twosiderender = twosiderender; }
        void setGlobalillum(bool globalillum) { this->globalillum = globalillum; }
        void setSamplingTechnique(string samplingTechnique) { this->samplingTechnique = samplingTechnique; }

        // Getters
        const string* getFilename() { return &filename; }
        const unsigned int* getWidth() { return &width; }
        const unsigned int* getHeight() { return &height; }
        const float* getFov() { return &fov; }
        const Eigen::Vector3f* getUp() { return &up; }
        const Eigen::Vector3f* getLookat() { return &lookat; }
        const Eigen::Vector3f* getAi() { return &ai; }
        const Eigen::Vector3f* getBkc() { return &bkc; }
        const Eigen::Vector3f* getCentre() { return &centre; }
        const float* getMaxbounces() { return &maxbounces; }
        const float* getProbterminate() { return &probterminate; }
        Eigen::Vector3f* getRaysperpixel() { return &raysperpixel; }
        const bool* getAntialiasing() { return &antialiasing; }
        const bool* getTwosiderender() { return &twosiderender; }
        const bool* getGlobalillum() { return &globalillum; }
        const string* getSamplingTechnique() { return &samplingTechnique; }

    private:
        // Mandatory members
        string filename;
        unsigned int width, height;
        float fov;
        Eigen::Vector3f up, lookat, ai, bkc, centre, raysperpixel;

        // Optional members
        float maxbounces, probterminate;
        bool antialiasing, twosiderender, globalillum;
        string samplingTechnique;
};
#endif