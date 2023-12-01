#include "Output.h"

// Output default constructor
Output::Output() {
    filename = "";
    width = 0;
    height = 0;
    fov = 0;
    up = Eigen::Vector3f(0,0,0);
    lookat = Eigen::Vector3f(0,0,0);
    ai = Eigen::Vector3f(0,0,0);
    bkc = Eigen::Vector3f(0,0,0);
    centre = Eigen::Vector3f(0,0,0);
    raysperpixel = Eigen::Vector3f(0,0,0);
    antialiasing = false;
    twosiderender = false;
    globalillum = false;
    maxbounces = 0;
    probterminate = 0;
    samplingTechnique = "uniform"; // default
}

// Output copy constructor
Output::Output(const Output &o) {
    filename = o.filename;
    width = o.width;
    height = o.height;
    fov = o.fov;
    up = o.up;
    lookat = o.lookat;
    ai = o.ai;
    bkc = o.bkc;
    centre = o.centre;
    raysperpixel = o.raysperpixel;
    antialiasing = o.antialiasing;
    twosiderender = o.twosiderender;
    globalillum = o.globalillum;
    maxbounces = o.maxbounces;
    probterminate = o.probterminate;
    samplingTechnique = o.samplingTechnique;
}