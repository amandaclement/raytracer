#include "Light.h"

// Light default constructor
Light::Light() {
    this->type = "";
    this->id = Eigen::Vector3f(0,0,0);
    this->is = Eigen::Vector3f(0,0,0);
    this->normal = Eigen::Vector3f(0,0,0);
    this->use = true; // default value is true
}

// Light copy constructor
Light::Light(const Light &l) {
    this->type = l.type;
    this->id = l.id;
    this->is = l.is;
    this->normal = l.normal;
    this->use = l.use;
}

// Method to return the closest geometry that the ray intersects with, if any
Geometry* Light::closestGeometry(Ray ray, vector<Geometry*> geometries) {
    // Pointer that will point to geometry closest to camera
    Geometry* geoPtr = nullptr;

    // Check for initial intersection
    bool initialIntersection = false;
    float minT = 0;
    int index = 0;

    // Need to only check if geometry is in front of light (e.g. ceiling is behind light)
    for (int k = 0; k < geometries.size(); k++) {
        if (geometries[k]->intersects(ray)) {
            // If it's the first geometry the ray intersects with, assume it has the smaller t scalar value
            if (initialIntersection == false) {
                minT = *geometries[k]->getT();
                index = k;
                initialIntersection = true;
                geoPtr = geometries[k];
            }
            // If it's not the first geometry the ray intersects with, compare the t scalar values to use the smallest one (representing the frontmost geometry)
            else if (*geometries[k]->getT() < minT) {
                minT = *geometries[k]->getT();
                index = k;
                geoPtr = geometries[k];
            }
        }
    }

    // Return the ptr, which may be a nullptr if no intersection
    return 
        geoPtr;
}

// Phong (direct) illumination method to be called for each light source contribution
Eigen::Vector3f Light::phongContribution(vector<Geometry*> geometries, Geometry* closestGeo, Output* output, Light* light, Eigen::Vector3f lightPoint, Eigen::Vector3f surfaceNormal, Ray ray) {
    // Variables for diffuse and specular calculations
    float diffuseR = 0.0f, diffuseG = 0.0f, diffuseB = 0.0f, specularR = 0.0f, specularG = 0.0f, specularB = 0.0f, lambertian = 0.0f, shininess = 0.0f, spec = 0.0f, specularAngle = 0.0f;
    Eigen::Vector3f L(0.0f,0.0f,0.0f), V(0.0f,0.0f,0.0f), H(0.0f,0.0f,0.0f);

    // Compute intersection point between ray and geometry
    Eigen::Vector3f intersectionPoint = *output->getCentre() + *closestGeo->getT() * ray.getDir();

    // Light direction calculation = unit vector between point and light = ||light source centre - intersection point|| 
    L = (lightPoint - intersectionPoint).normalized();

    // Shadow testing: cast a ray from point to light source, and check if we intersect any geometry (besides the one we're currently computing the light for) in between
    // Only if there's no geometry in-between do we add diffuse and specular to total light color
    // Construct the light ray: from point in direction of light
    ray = Ray(intersectionPoint, L);

    // Computie distance between light source point and intersection point (point on geometry)
    double pointToLightDist = sqrt((lightPoint[0] - intersectionPoint[0])*(lightPoint[0] - intersectionPoint[0]) + (lightPoint[1] - intersectionPoint[1])*(lightPoint[1] - intersectionPoint[1]) + (lightPoint[2] - intersectionPoint[2])*(lightPoint[2] - intersectionPoint[2]));
 
    // Assume there's no shadow, and then check for intersection with geometry along the ray we just computed
    bool inShadow = false;
    for (int k = 0; k < geometries.size(); k++) { 
        // When checking if point is in shadow, need to check if between intersectionPoint and light, we intersect any other geometry
        // If we do intersect, need to ensure this geometry is in front of light source
        // To check this, we can compare scalar t (distance between ray origin (intersectionPoint) and the geometry) to the distance between the intersectionPoint and the light source. If t is less than pointToLightDist, it's in range (so it's between intersectionPoint and light and therefore cannot be behind the light)
        if (geometries[k] != closestGeo && geometries[k]->intersects(ray) && *geometries[k]->getT() < pointToLightDist) {  
            // As soon as there's an intersection found, we can exit loop as we know it's in shadow
            inShadow = true;
            break;
        }
    }

    // Only adding diffuse and specular to final light value if the pixel isn't in shadow
    if (!inShadow) {
        // Lambertian calculation 
        // Angle of incidence (cos(angle)) = L.N where L = light direction and N = surface normal
        lambertian = max(surfaceNormal.dot(L), (float)0.0);

        // DIFFUSE LIGHT CALCULATION
        // diffuse light intensity (is) * diffuse reflection coefficient (kd) * lambertian (N.L) * diffuse light color (dc)
        diffuseR = (*light->getId())[0] * *closestGeo->getKd() * lambertian * (*closestGeo->getDc())[0];
        diffuseG = (*light->getId())[1] * *closestGeo->getKd() * lambertian * (*closestGeo->getDc())[1];
        diffuseB = (*light->getId())[2] * *closestGeo->getKd() * lambertian * (*closestGeo->getDc())[2]; 

        // SPECULAR LIGHT CALCULATION
        // specular light intensity (is) * specular reflection coefficient (ks) * (H.N)^shininess * specular light color (sc)
        // specular angle = max(H.N,0) where H = halfway vector and N = surface normal
        shininess = *closestGeo->getPc(); // shininess = specular reflection exponent 
        spec = 0.0;
        if (lambertian > 0.0) {
            V = (-intersectionPoint).normalized(); // V = view direction (vector to viewer)
            H = (L + V).normalized();              // H = halfway vector between light direction and view direction
            specularAngle = max((H.dot(surfaceNormal)), (float)0.0);
            spec = pow(specularAngle, shininess);
        }
        specularR = (*light->getIs())[0] * *closestGeo->getKs() * spec * (*closestGeo->getSc())[0];
        specularG = (*light->getIs())[1] * *closestGeo->getKs() * spec * (*closestGeo->getSc())[1];
        specularB = (*light->getIs())[2] * *closestGeo->getKs() * spec * (*closestGeo->getSc())[2];
    }

    // Return the light contribution of this light source
    return Eigen::Vector3f((diffuseR + specularR), (diffuseG + specularG), (diffuseB + specularB));
}

// Phong (direct) illumination model
Eigen::Vector3f Light::phong(vector<Geometry*> geometries, Output* output, vector<Light*> lights, Ray ray, Eigen::Vector3f pixel, Eigen::Vector3f right) {

    Eigen::Vector3f surfaceNormal(0.0f,0.0f,0.0f), lightPoint(0.0f,0.0f,0.0f);

    // Returns closest geometry if there is intersection, else returns nullptr
    Geometry* closestGeo = closestGeometry(ray, geometries);

    // If closestGeo returns a nullptr, there's no intersection so return background color
    if (closestGeo == nullptr)
        return {(*output->getBkc())[0],(*output->getBkc())[1],(*output->getBkc())[2]};

    // Else calculate lighting
    else {
        // Compute intersection point between ray and geometry
        Eigen::Vector3f intersectionPoint = *output->getCentre() + *closestGeo->getT() * ray.getDir();

        // Assign the surface normal based on geometry type
        if (*closestGeo->getType() == "sphere")
            // Sphere surface normal = ||intersection point - sphere centre||
            surfaceNormal = (intersectionPoint - *(dynamic_cast<Sphere*>(closestGeo))->getCentre()).normalized();
        else if (*closestGeo->getType() == "rectangle")
            surfaceNormal = *(dynamic_cast<Rectangle*>(closestGeo))->getNormal();

        // AMBIENT LIGHT CALCULATION
        // ambient intensity (ai) * ambient reflection coefficient (ka) * ambient light color (ac)
        float totalR = (*output->getAi())[0] * *closestGeo->getKa() * (*closestGeo->getAc())[0];
        float totalG = (*output->getAi())[1] * *closestGeo->getKa() * (*closestGeo->getAc())[1];
        float totalB = (*output->getAi())[2] * *closestGeo->getKa() * (*closestGeo->getAc())[2];

        // Iterate over all lights in scene to compute diffuse and specular contributions
        for (int i = 0; i < lights.size(); i++) {
            // Only calculate the light contribution if use is true
            if (*lights[i]->getUse()) {
                // AREA LIGHT CALCULATIONS FOR AREA LIGHTS TREATED AS POINT LIGHTS
                if (*lights[i]->getType() == "area") {
                    // If usecenter is true, we simply treat the area light as a point light source
                    if (*static_cast<Area*>(lights[i])->getUsecenter()) {
                        // Compute the midpoint of the area light plane by taking average of the x, y, and z coordinates of the four input points separately and assigning the result as a vector with three elements to lightPoint
                        lightPoint = {((*static_cast<Area*>(lights[i])->getP1())[0] + (*static_cast<Area*>(lights[i])->getP2())[0] + (*static_cast<Area*>(lights[i])->getP3())[0] + (*static_cast<Area*>(lights[i])->getP4())[0])/4, ((*static_cast<Area*>(lights[i])->getP1())[1] + (*static_cast<Area*>(lights[i])->getP2())[1] + (*static_cast<Area*>(lights[i])->getP3())[1] + (*static_cast<Area*>(lights[i])->getP4())[1])/4, ((*static_cast<Area*>(lights[i])->getP1())[2] + (*static_cast<Area*>(lights[i])->getP2())[2] + (*static_cast<Area*>(lights[i])->getP3())[2] + (*static_cast<Area*>(lights[i])->getP4())[2])/4};
                        
                        // Add the contribution of the light source to the total
                        Eigen::Vector3f lightContribution = phongContribution(geometries, closestGeo, output, lights[i], lightPoint, surfaceNormal, ray);
                        totalR += lightContribution[0];
                        totalG += lightContribution[1];
                        totalB += lightContribution[2];
                    } 

                    // AREA LIGHT CALCULATIONS FOR AREA LIGHTS TREATED AS AREA LIGHTS
                    else {
                        // Number of rays for each pixel for area light stratified sampling
                        int samples = *static_cast<Area*>(lights[i])->getSamples();

                        // Compute the width and height of the rectangular area light
                        Eigen::Vector3f width = *static_cast<Area*>(lights[i])->getP2() - *static_cast<Area*>(lights[i])->getP1();
                        Eigen::Vector3f height = *static_cast<Area*>(lights[i])->getP4() - *static_cast<Area*>(lights[i])->getP1();

                        // To multiply each grid point contribution by when adding it to the total light value
                        float weight = 1.0f / (samples * samples);

                        // Iterate over the samples grid, passing the midpoint of each grid cell to the phongContribution function
                        for (int j = 0; j < samples; j++) {
                            for (int k = 0; k < samples; k++) {
                                // Compute the center point of the current grid cell 
                                lightPoint = *static_cast<Area*>(lights[i])->getP1() + width * ((j + 0.5f) / samples) + height * ((k + 0.5f) / samples);

                                // Add the contribution of the light source to the total
                                Eigen::Vector3f lightContribution = phongContribution(geometries, closestGeo, output, lights[i], lightPoint, surfaceNormal, ray);
                                totalR += weight * lightContribution[0];
                                totalG += weight * lightContribution[1];
                                totalB += weight * lightContribution[2];
                            }    
                        }
                    }
                }

                // POINT LIGHT CALCULATIONS
                if (*lights[i]->getType() == "point") {
                    // For point light, the light point we use for the computation is the centre of the light
                    lightPoint = static_cast<Point*>(lights[i])->getCentre();

                    // Add the contribution of the light source to the total
                    Eigen::Vector3f lightContribution = phongContribution(geometries, closestGeo, output, lights[i], lightPoint, surfaceNormal, ray);
                    totalR += lightContribution[0];
                    totalG += lightContribution[1];
                    totalB += lightContribution[2];
                }
            }
        }

        // Return the final color of pixel as combo of ambient + diffuse + specular lighting for R, G and B, clamped to [0-1] to limit amount of light we let in
        return Eigen::Vector3f(min(float(1.0), max(totalR,float(0.0))), min(float(1.0), max(totalG,float(0.0))), min(float(1.0), max(totalB,float(0.0))));
    }
}

// Method to generate a random direction on a hemisphere that's centered around the surface normal and weighed by cosine
Eigen::Vector3f Light::hemisphereSample(Eigen::Vector3f surfaceNormal)
{
    // Generate two random floats in range [0-1]
    double rand1 = rand() / (RAND_MAX + 1.0);
    double rand2 = rand() / (RAND_MAX + 1.0);

    // Compute radius and angle of the point on the hemisphere
    double radius = sqrt(1 - rand1*rand1);
    double angle = 2 * EIGEN_PI * rand2;

    // Compute a vector that's perpendicular to the surface normal
    Eigen::Vector3f tangent;
    if (surfaceNormal[0] != 0.0 || surfaceNormal[1] != 0.0)
        tangent = Eigen::Vector3f(-surfaceNormal[1], surfaceNormal[0], 0.0);
    else
        tangent = Eigen::Vector3f(0.0, -surfaceNormal[2], surfaceNormal[1]);

    // Compute two vectors that are perpendicular
    Eigen::Vector3f x = radius * cos(angle) * tangent;
    Eigen::Vector3f y = radius * sin(angle) * normal.cross(tangent);

    // Compute the new direction using the three vectors
    Eigen::Vector3f direction = x + y + rand1 * surfaceNormal;
    return direction;
}

// Global illumination model
Eigen::Vector3f Light::globalillum(vector<Geometry*> geometries, Output* output, Light* light, Ray ray, int numbounces, int maxbounces, Eigen::Vector3f bgColor) {
    // If numbounces is equal to or greater than 1, return bg color
    if (numbounces >= maxbounces)
        return { 0.0, 0.0, 0.0 };

    // If numbounces is at least 1, use russian roulette (probterminate) to cut the loop
    if (numbounces > 0 && ((float)rand()/(RAND_MAX)) <= *output->getProbterminate())
        return { 0.0, 0.0, 0.0 };

    // Returns closest geometry if there is intersection, else returns nullptr
    Geometry* closestGeo = closestGeometry(ray, geometries);

    // If closestGeo returns a nullptr, there's no intersection so return background color
    if (closestGeo == nullptr)
        return {bgColor[0], bgColor[1], bgColor[2]};

    // Compute the intersection point between the ray and closest geometry
    Eigen::Vector3f point = ray.getCentre() + *closestGeo->getT() * ray.getDir();

    // Compute the surface normal of the geometry
    Eigen::Vector3f surfaceNormal(0.0f,0.0f,0.0f);
    
    // Update surface normal
    if (*closestGeo->getType() == "sphere") 
        surfaceNormal = (point - *(dynamic_cast<Sphere*>(closestGeo))->getCentre()).normalized();
    else if (*closestGeo->getType() == "rectangle")
        surfaceNormal = *(dynamic_cast<Rectangle*>(closestGeo))->getNormal();

    // Generate a bounce ray in random direction on a hemisphere that's centered around the surface normal and weighed by cosine
    Ray bounceRay(point + surfaceNormal, hemisphereSample(surfaceNormal));
    
    // Diffuse color * diffuse intensity
    Eigen::Vector3f BRDF = {(*closestGeo->getDc())[0] * (*closestGeo->getKd()), (*closestGeo->getDc())[1] * (*closestGeo->getKd()), (*closestGeo->getDc())[2] * (*closestGeo->getKd())}; 

    // Compute the light position for point light (if we have an area, assume usecenter is true and treat it like a point light)
    Eigen::Vector3f lightPoint(0.0f,0.0f,0.0f);
    if (*light->getType() == "point")
        lightPoint = static_cast<Point*>(light)->getCentre();
    else if (*static_cast<Area*>(light)->getUsecenter())
        lightPoint = {((*static_cast<Area*>(light)->getP1())[0] + (*static_cast<Area*>(light)->getP2())[0] + (*static_cast<Area*>(light)->getP3())[0] + (*static_cast<Area*>(light)->getP4())[0])/4, ((*static_cast<Area*>(light)->getP1())[1] + (*static_cast<Area*>(light)->getP2())[1] + (*static_cast<Area*>(light)->getP3())[1] + (*static_cast<Area*>(light)->getP4())[1])/4, ((*static_cast<Area*>(light)->getP1())[2] + (*static_cast<Area*>(light)->getP2())[2] + (*static_cast<Area*>(light)->getP3())[2] + (*static_cast<Area*>(light)->getP4())[2])/4};
    
    // Compute light direction to get cosTheta
    Eigen::Vector3f lightDirection = (lightPoint - point).normalized();
    float pointToLightDist = sqrt((lightPoint[0] - point[0])*(lightPoint[0] - point[0]) + (lightPoint[0] - point[1])*(lightPoint[1] - point[1]) + (lightPoint[2] - point[2])*(lightPoint[2] - point[2]));
    float cosTheta = max(surfaceNormal.dot(lightDirection), float(0.0));

    // Assume there's no shadow, and then shoot ray from current point to the light and see if there's anything in between causing this pixel to be shadowed
    bool inShadow = false;

    // Construct ray between intersection point and light for shadow check
    Ray shadowRay(point, lightDirection);

    for (int k = 0; k < geometries.size(); k++) { 
        // When checking if point is in shadow, need to check if between intersectionPoint and light, we intersect any other geometry
        if (geometries[k] != closestGeo && geometries[k]->intersects(shadowRay) && *geometries[k]->getT() < pointToLightDist) {  
            // As soon as there's an intersection found, we can exit loop as we know it's in shadow
            inShadow = true;
            break;
        }
    }

    Eigen::Vector3f lightIntensity(0.0,0.0,0.0);

    // Only computing light intensity (color) if the pixel isn't in shadow
    if (!inShadow)
        lightIntensity = {((*light->getId())[0]), ((*light->getId())[1]), ((*light->getId())[2])};

    // Recursively trace reflected light sources for indirect light contributions
    Eigen::Vector3f indirectIllumination = globalillum(geometries, output, light, bounceRay, numbounces+1, maxbounces, bgColor);

    Eigen::Vector3f finalColor(0.0,0.0,0.0);

    // Rendering equation (has some issues, as shown by the output)
    finalColor[0] = BRDF[0] * (indirectIllumination[0] + (lightIntensity[0] * (cosTheta / EIGEN_PI)));
    finalColor[1] = BRDF[1] * (indirectIllumination[1] + (lightIntensity[1] * (cosTheta / EIGEN_PI)));
    finalColor[2] = BRDF[2] * (indirectIllumination[2] + (lightIntensity[2] * (cosTheta / EIGEN_PI)));

    // Apply gamma correction to brighten the image
    float gamma = 0.5f;
    finalColor[0] = pow(finalColor[0], gamma);
    finalColor[1] = pow(finalColor[1], gamma);
    finalColor[2] = pow(finalColor[2], gamma);
    
    // Return the final color as a result of both direct and indirect illumination
    return finalColor;  
}

// Point light default constructor
Point::Point() {
    this->centre = Eigen::Vector3f(0,0,0);
}

// Point light copy constructor
Point::Point(const Point &p) : Light(p) {
    this->centre = p.centre;
}

// Point light parameterized constructor
Point::Point(Eigen::Vector3f centre) {
    this->centre = centre;
}

// Area light default constructor
Area::Area() {
    this->p1 = Eigen::Vector3f(0,0,0);
    this->p2 = Eigen::Vector3f(0,0,0);
    this->p3 = Eigen::Vector3f(0,0,0);
    this->p4 = Eigen::Vector3f(0,0,0);
    this->samples = 10;      // default to 10
    this->usecenter = false; // default value is false
}

// Area light copy constructor
Area::Area(const Area &a) : Light(a) {
    this->p1 = a.p1;
    this->p2 = a.p2;
    this->p3 = a.p3;
    this->p4 = a.p4;
    this->samples = a.samples;
    this->usecenter = a.usecenter;
}

// Area light parameterized constructor
Area::Area(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, Eigen::Vector3f p4, unsigned int samples) {
    this->p1 = p1;
    this->p2 = p2;
    this->p3 = p3;
    this->p4 = p4;
    this->samples = samples;
}