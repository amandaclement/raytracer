#include "RayTracer.h"

// Raytracer constructor that takes a json file containing info about the scene as its argument
RayTracer::RayTracer(nlohmann::json j) {
    // Parse the json content and assign values to the appropriate variables
    parseGeometry(j);
    parseLights(j);
    parseOutput(j);
}

// Raytracer's render method for rendering the scene
int RayTracer::render() {
    // Iterate over outputs, generating a ppm file for each output
    for (int o = 0; o < outputs.size(); o++) {
        // Image size is stored in an array of two unsigned int vars
        unsigned int imageWidth = *outputs[o]->getWidth();      // getSize() returns width
        unsigned int imageHeight = *outputs[o]->getHeight(); // getSize()+1 return height
        Eigen::Vector3f origin = *outputs[o]->getCentre();
        Eigen::Vector3f lookAt = *outputs[o]->getLookat();
        Eigen::Vector3f up = *outputs[o]->getUp();
        float fov = *outputs[o]->getFov();
        Eigen::Vector3f bgColor = *outputs[o]->getBkc();
        string samplingTechnique = *outputs[o]->getSamplingTechnique();
        Eigen::Vector3f raysperpixel = *outputs[o]->getRaysperpixel();

        // If global illumination is used, ignore antialiasing
        if (*outputs[o]->getGlobalillum())
            outputs[o]->setAntialiasing(false);

        // Prepare the buffer
        std::vector<double> buffer(3*imageWidth*imageHeight);

        // Define a pixel's size (height and width of a pixel are equal as it's a square)
        float pixelSize = (2*tan(fov/2))/imageHeight;

        // Note: up and lookat vectors have have been normalized

        // Compute the right vector, which is the cross product of the lookat and up vectors
        Eigen::Vector3f right = lookAt.cross(up).normalized();

        // A represents middle of the pixel grid (the intersection between the lookat vector and image)
        Eigen::Vector3f A = origin + lookAt;

        // B represents the mid top of the pixel grid
        Eigen::Vector3f B = A + tan(fov/2)*(up);

        // C represents the top left corner of the grid
        Eigen::Vector3f C = B - (((imageWidth/2)*pixelSize)*right);

        // Light pointer to access the light methods
        Light* lightAccess;

        // Nested for loop to iterate over each pixel on screen
        for (int j = 0; j < imageHeight; j++) {
            for (int i = 0; i < imageWidth; i++) {
                // Starting from point C, which is the top left corner of the grid, we find the current point 
                Eigen::Vector3f point = C + (i*pixelSize + pixelSize/2) * right - (j*pixelSize + pixelSize/2) * up;

                // Since we now have two points on the line, being the origin (centre) and point (P(i,j)), we can compute the direction of the ray
                Eigen::Vector3f dir = (point - origin).normalized();

                // Construct the ray
                Ray ray = Ray(origin, dir);

                // Use global illumination
                if (*outputs[o]->getGlobalillum()) {
                    // We're only considering one light source for global illumination so find the first valid one
                    int lightIndex = 0;
                    for (int l = 0; l < lights.size(); l++)
                        if (*lights[l]->getUse() == true)
                            lightIndex = l;

                    // STRATIFIED SAMPLING
                    if (samplingTechnique == "stratified") {
                        float gridWidth = float(raysperpixel[0]);     
                        float gridHeight = gridWidth; 
                        float numSamples = float(raysperpixel[1]);
                            
                        Eigen::Vector3f lightTotal(0.0,0.0,0.0);

                        for (int x = 0; x < gridWidth; x++) {
                            for (int y = 0; y < gridHeight; y++) {
                                for (int z = 0; z < numSamples; z++) {
                                    // Generate random subpixel location within within the subpixel grid
                                    float offsetX = ((float)rand() / RAND_MAX - 0.5f + (float)x / gridWidth) * pixelSize;
                                    float offsetY = ((float)rand() / RAND_MAX - 0.5f + (float)y / gridHeight) * pixelSize;

                                    // Point is center of current pixel
                                    Eigen::Vector3f subpixel = point + offsetX * right + offsetY * up;

                                    // Since we now have two points on the line, we can compute the direction of the ray
                                    Eigen::Vector3f subdir = (subpixel - origin).normalized();

                                    // Construct the ray 
                                    Ray subray(origin, subdir);

                                    // Pass the ray for light calculations
                                    Eigen::Vector3f light = lightAccess->globalillum(geometries, outputs[o], lights[lightIndex], ray, 0, *outputs[o]->getMaxbounces(), bgColor);

                                    // Accumulate the contributions
                                    lightTotal[0] += light[0];
                                    lightTotal[1] += light[1];
                                    lightTotal[2] += light[2];
                                }
                            }
                        }
                        // Apply final color, with contributions weighed based on on # rays per pixel
                        buffer[3*j*imageWidth+3*i+0] = lightTotal[0]/(gridWidth*gridHeight*numSamples);
                        buffer[3*j*imageWidth+3*i+1] = lightTotal[1]/(gridWidth*gridHeight*numSamples);
                        buffer[3*j*imageWidth+3*i+2] = lightTotal[2]/(gridWidth*gridHeight*numSamples); 
                    } 

                    // STRATIFIED SAMPLING USING GRID
                    else if (samplingTechnique == "grid") {
                        float cellWidth = float(raysperpixel[0]);   
                        float cellHeight = float(raysperpixel[1]); 
                        float numSamples = float(raysperpixel[2]); 
                            
                        Eigen::Vector3f lightTotal(0.0,0.0,0.0);

                        for (int x = 0; x < cellWidth; x++) {
                            for (int y = 0; y < cellHeight; y++) {
                                for (int z = 0; z < numSamples; z++) {
                                    // Generate random subpixel location within within the subpixel grid
                                    float offsetX = ((float)rand() / RAND_MAX - 0.5f + (float)x / cellWidth) * pixelSize;
                                    float offsetY = ((float)rand() / RAND_MAX - 0.5f + (float)y / cellHeight) * pixelSize;

                                    // Point is center of current pixel
                                    Eigen::Vector3f subpixel = point + offsetX * right + offsetY * up;

                                    // Since we now have two points on the line, we can compute the direction of the ray
                                    Eigen::Vector3f subdir = (subpixel - origin).normalized();

                                    // Construct the ray 
                                    Ray subray(origin, subdir);

                                    // Pass the ray for light calculations
                                    Eigen::Vector3f light = lightAccess->globalillum(geometries, outputs[o], lights[lightIndex], ray, 0, *outputs[o]->getMaxbounces(), bgColor);

                                    // Accumulate the contributions
                                    lightTotal[0] += light[0];
                                    lightTotal[1] += light[1];
                                    lightTotal[2] += light[2];
                                }
                            }
                        }
                        // Apply final color, with contributions weighed based on on # rays per pixel
                        buffer[3*j*imageWidth+3*i+0] = lightTotal[0]/(cellWidth*cellHeight*numSamples);
                        buffer[3*j*imageWidth+3*i+1] = lightTotal[1]/(cellWidth*cellHeight*numSamples);
                        buffer[3*j*imageWidth+3*i+2] = lightTotal[2]/(cellWidth*cellHeight*numSamples); 
                    } 

                    // UNIFORM RANDOM SAMPLING (DEFAULT)
                    else {
                        float numSamples = float(raysperpixel[0]);
                        Eigen::Vector3f lightTotal(0.0,0.0,0.0);

                        for (int x = 0; x < numSamples; x++) {
                            // Generate random subpixel location within the pixel
                            float offsetX = ((float)rand() / RAND_MAX - 0.5f) * pixelSize;
                            float offsetY = ((float)rand() / RAND_MAX - 0.5f) * pixelSize;

                            // Point is center of current pixel
                            Eigen::Vector3f subpixel = point + offsetX * right + offsetY * up;

                            // Since we now have two points on the line, we can compute the direction of the ray
                            Eigen::Vector3f subdir = (subpixel - origin).normalized();

                            // Construct the ray 
                            Ray subray(origin, subdir);

                            // Pass the ray for light calculations
                            Eigen::Vector3f light = lightAccess->globalillum(geometries, outputs[o], lights[lightIndex], ray, 0, *outputs[o]->getMaxbounces(), bgColor);
                            
                            // Accumulate the contributions
                            lightTotal[0] += light[0];
                            lightTotal[1] += light[1];
                            lightTotal[2] += light[2];
                        }
                        // Apply final color, with contributions weighed based on on # rays per pixel
                        buffer[3*j*imageWidth+3*i+0] = lightTotal[0]/numSamples;
                        buffer[3*j*imageWidth+3*i+1] = lightTotal[1]/numSamples;
                        buffer[3*j*imageWidth+3*i+2] = lightTotal[2]/numSamples;
                    }
                }  

                // Use direct illumination (Phong model)
                else {
                    // Apply basic Phong model if no antialiasing
                    if (*outputs[o]->getAntialiasing() == false) {
                        Eigen::Vector3f light = lightAccess->phong(geometries, outputs[o], lights, ray, point, right);

                        // Applying color via ambient + diffuse + specular lighting to pixel
                        buffer[3*j*imageWidth+3*i+0] = light[0];
                        buffer[3*j*imageWidth+3*i+1] = light[1];
                        buffer[3*j*imageWidth+3*i+2] = light[2];
                    } 

                    // ANTIALIASING CALCULATIONS
                    // Only apply antialiasting if we're not using global illumination
                    else if (*outputs[o]->getGlobalillum() == false) {
                        // STRATIFIED SAMPLING
                        if (samplingTechnique == "stratified") {
                            float gridWidth = float(raysperpixel[0]); 
                            float gridHeight = gridWidth; 
                            float numSamples = float(raysperpixel[1]);
                            
                            Eigen::Vector3f lightTotal(0.0,0.0,0.0);

                            for (int x = 0; x < gridWidth; x++) {
                                for (int y = 0; y < gridHeight; y++) {
                                    for (int z = 0; z < numSamples; z++) {
                                        // Generate random subpixel location within within the subpixel grid
                                        float offsetX = ((float)rand() / RAND_MAX - 0.5f + (float)x / gridWidth) * pixelSize;
                                        float offsetY = ((float)rand() / RAND_MAX - 0.5f + (float)y / gridHeight) * pixelSize;

                                        // Point is center of current pixel
                                        Eigen::Vector3f subpixel = point + offsetX * right + offsetY * up;

                                        // Since we now have two points on the line, we can compute the direction of the ray
                                        Eigen::Vector3f subdir = (subpixel - origin).normalized();

                                        // Construct the ray 
                                        Ray subray(origin, subdir);

                                        // Pass the ray for light calculations
                                        Eigen::Vector3f light = lightAccess->phong(geometries, outputs[o], lights, subray, point, right);
                                        lightTotal[0] += light[0];
                                        lightTotal[1] += light[1];
                                        lightTotal[2] += light[2];
                                    }
                                }
                            }
                            // Apply final color, with contributions weighed based on on # rays per pixel
                            buffer[3*j*imageWidth+3*i+0] = lightTotal[0]/(gridWidth*gridHeight*numSamples);
                            buffer[3*j*imageWidth+3*i+1] = lightTotal[1]/(gridWidth*gridHeight*numSamples);
                            buffer[3*j*imageWidth+3*i+2] = lightTotal[2]/(gridWidth*gridHeight*numSamples); 
                        }

                        // STRATIFIED SAMPLING USING GRID
                        else if (samplingTechnique == "grid") {
                            float cellWidth = float(raysperpixel[0]);
                            float cellHeight = float(raysperpixel[1]); 
                            float numSamples = float(raysperpixel[2]);

                            Eigen::Vector3f lightTotal(0.0,0.0,0.0);

                            for (int x = 0; x < cellWidth; x++) {
                                for (int y = 0; y < cellHeight; y++) {
                                    for (int z = 0; z < numSamples; z++) {
                                        // Generate random subpixel location within within the subpixel grid
                                        float offsetX = ((float)rand() / RAND_MAX - 0.5f + (float)x / cellWidth) * pixelSize;
                                        float offsetY = ((float)rand() / RAND_MAX - 0.5f + (float)y / cellHeight) * pixelSize;

                                        // Point is center of current pixel
                                        Eigen::Vector3f subpixel = point + offsetX * right + offsetY * up;

                                        // Since we now have two points on the line, we can compute the direction of the ray
                                        Eigen::Vector3f subdir = (subpixel - origin).normalized();

                                        // Construct the ray 
                                        Ray subray(origin, subdir);

                                        // Pass the ray for light calculations
                                        Eigen::Vector3f light = lightAccess->phong(geometries, outputs[o], lights, subray, point, right);
                                        lightTotal[0] += light[0];
                                        lightTotal[1] += light[1];
                                        lightTotal[2] += light[2];
                                    }
                                }
                            }
                            // Apply final color, with contributions weighed based on on # rays per pixel
                            buffer[3*j*imageWidth+3*i+0] = lightTotal[0]/(cellWidth*cellHeight*numSamples);
                            buffer[3*j*imageWidth+3*i+1] = lightTotal[1]/(cellWidth*cellHeight*numSamples);
                            buffer[3*j*imageWidth+3*i+2] = lightTotal[2]/(cellWidth*cellHeight*numSamples); 
                        }

                        // UNIFORM RANDOM SAMPLING (DEFAULT)
                        else  {
                            float numSamples = float(raysperpixel[0]); 
                            Eigen::Vector3f lightTotal(0.0,0.0,0.0);

                            for (int x = 0; x < numSamples; x++) {
                                // Generate random subpixel location within the pixel
                                float offsetX = ((float)rand() / RAND_MAX - 0.5f) * pixelSize;
                                float offsetY = ((float)rand() / RAND_MAX - 0.5f) * pixelSize;

                                // Point is center of current pixel
                                Eigen::Vector3f subpixel = point + offsetX * right + offsetY * up;

                                // Since we now have two points on the line, we can compute the direction of the ray
                                Eigen::Vector3f subdir = (subpixel - origin).normalized();

                                // Construct the ray 
                                Ray subray(origin, subdir);

                                // Pass the ray for light calculations
                                Eigen::Vector3f light = lightAccess->phong(geometries, outputs[o], lights, subray, point, right);
                                lightTotal[0] += light[0];
                                lightTotal[1] += light[1];
                                lightTotal[2] += light[2];
                            }

                            // Apply final color, with contributions weighed based on on # samples per pixel
                            buffer[3*j*imageWidth+3*i+0] = lightTotal[0]/numSamples;
                            buffer[3*j*imageWidth+3*i+1] = lightTotal[1]/numSamples;
                            buffer[3*j*imageWidth+3*i+2] = lightTotal[2]/numSamples;
                        }
                    }
                }
            }
        }
        // Save the ppm file
        save_ppm(*outputs[o]->getFilename(), buffer, imageWidth, imageHeight);
    }
    return 0;
}

// RayTracer's run method simply calls the render() method, which is the one responsible for writing to the buffer
void RayTracer::run() {
    render();
}

// PARSE METHODS
// Raytracer's parseGeometry method for parsing the geometry-related content from the json file, and assigning the data to the appropriate variables
void RayTracer::parseGeometry(nlohmann::json& j) {
    cout << "Parsing geometry" << endl;
    int gc = 0;

    // Use iterator to read-in array types from JSON file
    for (auto itr = j["geometry"].begin(); itr!= j["geometry"].end(); itr++) {
        // First read type to know if we're dealing with a sphere or a rectangle
        string type;

        if (itr->contains("type"))
            type = (*itr)["type"].get<string>();

        // If type is sphere, assign values to a sphere object's variables and then push a Geometry pointer pointing to this sphere into geometries array
         // If type is sphere, assign values to a sphere object's variables and then push a Geometry pointer pointing to this sphere into geometries array
        if (type == "sphere") {
            Sphere s;
            s.setType(type);

            if (itr->contains("centre")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["centre"].begin(); itr2!= (*itr)["centre"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                s.setCentre(temp);
            }

            if (itr->contains("radius"))
                s.setRadius((*itr)["radius"].get<float>());

            if (itr->contains("ac")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["ac"].begin(); itr2!= (*itr)["ac"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                s.setAc(temp);
            }

            if (itr->contains("dc")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["dc"].begin(); itr2!= (*itr)["dc"].end(); itr2++)
                    if (i < 3)
                        temp[i++] = (*itr2).get<float>();
                s.setDc(temp);
            }

            if (itr->contains("sc")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["sc"].begin(); itr2!= (*itr)["sc"].end(); itr2++) 
                        temp[i++] = (*itr2).get<float>();
                s.setSc(temp);
            }

            if (itr->contains("ka"))
                s.setKa((*itr)["ka"].get<float>());

            if (itr->contains("kd"))
                s.setKd((*itr)["kd"].get<float>());

            if (itr->contains("ks"))
                s.setKs((*itr)["ks"].get<float>());

            if (itr->contains("pc"))
                s.setPc((*itr)["pc"].get<float>());
            
            if (itr->contains("visible")) 
                s.setVisible((*itr)["visible"].get<bool>());
            else 
                s.setVisible(true);

            geometries.push_back(new Sphere(s));
        } 
        
        // If type is rectangle, assign values to a rectangle object's variables and then push a Geometry pointer pointing to this rectangle into geometries array
        else if (type == "rectangle") {
            Rectangle r;
            r.setType(type);

            if (itr->contains("p1")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["p1"].begin(); itr2!= (*itr)["p1"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                r.setP1(temp);
            }

            if (itr->contains("p2")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["p2"].begin(); itr2!= (*itr)["p2"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                r.setP2(temp);
            }

            if (itr->contains("p3")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["p3"].begin(); itr2!= (*itr)["p3"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                r.setP3(temp);
            }

            if (itr->contains("p4")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["p4"].begin(); itr2!= (*itr)["p4"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                r.setP4(temp);
            }

            // The plane's normal is calculated using 3 points on the plane taking their cross products
            r.setNormal(((*r.getP2() - *r.getP1()).cross(*r.getP4() - *r.getP1())).normalized());

            if (itr->contains("ac")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["ac"].begin(); itr2!= (*itr)["ac"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                r.setAc(temp);
            }

            if (itr->contains("dc")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["dc"].begin(); itr2!= (*itr)["dc"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                r.setDc(temp);
            }

            if (itr->contains("sc")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["sc"].begin(); itr2!= (*itr)["sc"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                r.setSc(temp);
            }

            if (itr->contains("ka"))
                r.setKa((*itr)["ka"].get<float>());

            if (itr->contains("kd"))
                r.setKd((*itr)["kd"].get<float>());

            if (itr->contains("ks"))
                r.setKs((*itr)["ks"].get<float>());

            if (itr->contains("pc"))
                r.setPc((*itr)["pc"].get<float>());
            
            if (itr->contains("visible")) 
                r.setVisible((*itr)["visible"].get<bool>());
            else 
                r.setVisible(true);

            geometries.push_back(new Rectangle(r));
        } 
        ++gc;
    }
    cout << "\nWe have: " << gc << " objects!" << endl;
}

// Raytracer's parseLights method for parsing the light-related content from the json file, and assigning the data to the appropriate variables
void RayTracer::parseLights(nlohmann::json& j) {
    cout << "Parsing lights" << endl;
    int lc = 0;

    // Use iterator to read-in array types from JSON file
    for (auto itr = j["light"].begin(); itr!= j["light"].end(); itr++) {
        // First read type to know if we're dealing with a point light or an area light
        string type;

        if (itr->contains("type"))
            type = (*itr)["type"].get<string>();

        // If light type is point, assign values to a point object's variables and then push a Light pointer pointing to this point light into lights array
        if (type == "point") {
            Point p;
            p.setType(type);

            if (itr->contains("centre")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["centre"].begin(); itr2!= (*itr)["centre"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                p.setCentre(temp);
            }

            if (itr->contains("id")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["id"].begin(); itr2!= (*itr)["id"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                p.setId(temp);
            }

            if (itr->contains("is")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["is"].begin(); itr2!= (*itr)["is"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                p.setIs(temp);
            }

            if (itr->contains("use"))
                p.setUse((*itr)["use"].get<bool>());
            else 
                p.setUse(true);

            lights.push_back(new Point(p));
        }

        // If light type is area, assign values to an area object's variables and then push a Light pointer pointing to this area light into lights array
        else if (type == "area") {
            Area a;
            a.setType(type);

            if (itr->contains("p1")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["p1"].begin(); itr2!= (*itr)["p1"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                a.setP1(temp);
            }

            if (itr->contains("p2")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["p2"].begin(); itr2!= (*itr)["p2"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                a.setP2(temp);
            }

            if (itr->contains("p3")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["p3"].begin(); itr2!= (*itr)["p3"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                a.setP3(temp);
            }

            if (itr->contains("p4")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["p4"].begin(); itr2!= (*itr)["p4"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                a.setP4(temp);
            }

            // The area light (plane)'s normal is calculated using 3 points on the plane taking their cross products
            a.setNormal(((*a.getP2() - *a.getP1()).cross(*a.getP4() - *a.getP1())).normalized());

            if (itr->contains("id")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["id"].begin(); itr2!= (*itr)["id"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                a.setId(temp);
            }

            if (itr->contains("is")) {
                int i = 0;
                Eigen::Vector3f temp(0.0f,0.0f,0.0f);
                for (auto itr2 =(*itr)["is"].begin(); itr2!= (*itr)["is"].end(); itr2++)
                    temp[i++] = (*itr2).get<float>();
                a.setIs(temp);
            }

            if (itr->contains("n"))
                a.setSamples((*itr)["n"].get<unsigned int>());
            else
                a.setSamples(10);

            if (itr->contains("usecenter"))
                a.setUsecenter((*itr)["usecenter"].get<bool>());
            else 
                a.setUsecenter(false);

            if (itr->contains("use"))
                a.setUse((*itr)["use"].get<bool>());
            else 
                a.setUse(true);

            lights.push_back(new Area(a));
        }
        ++lc;
    }
    cout << "\nWe have: " << lc << " lights!" << endl;
}

// Raytracer's parseOutput method for parsing the output-related content from the json file, and assigning the data to the appropriate variables
void RayTracer::parseOutput(nlohmann::json& j) {
    cout << "Parsing output" << endl;
    int lc = 0;

    // Use iterator to read-in array types from JSON file
    for (auto itr = j["output"].begin(); itr!= j["output"].end(); itr++) {
        Output o;
        
        if (itr->contains("filename"))
            o.setFilename((*itr)["filename"].get<string>());

        if (itr->contains("size")) {
            int i = 0;
            static unsigned int temp[2];
            for (auto itr2 =(*itr)["size"].begin(); itr2!= (*itr)["size"].end(); itr2++)
                temp[i++] = (*itr2).get<unsigned int>();
            o.setWidth(temp[0]);
            o.setHeight(temp[1]);
        }
 
        // Normalize the vectors to get them in unit length
        if (itr->contains("lookat")) {
            int i = 0;
            Eigen::Vector3f temp(0.0f,0.0f,0.0f);
            for (auto itr2 =(*itr)["lookat"].begin(); itr2!= (*itr)["lookat"].end(); itr2++)
                temp[i++] = (*itr2).get<float>();
            o.setLookat(temp.normalized());
        }

        if (itr->contains("up")) {
            int i = 0;
            Eigen::Vector3f temp(0.0f,0.0f,0.0f);
            for (auto itr2 =(*itr)["up"].begin(); itr2!= (*itr)["up"].end(); itr2++)
                temp[i++] = (*itr2).get<float>();
            o.setUp(temp.normalized());
        }

        // FOV given in degrees but trig functions use radians so convert FOV to radians
        if (itr->contains("fov"))
            o.setFov(((*itr)["fov"].get<float>())*EIGEN_PI/180);

        if (itr->contains("centre")) {
            int i = 0;
            Eigen::Vector3f temp(0.0f,0.0f,0.0f);
            for (auto itr2 =(*itr)["centre"].begin(); itr2!= (*itr)["centre"].end(); itr2++)
                temp[i++] = (*itr2).get<float>();
            o.setCentre(temp);
        }

        if (itr->contains("ai")) {
            int i = 0;
            Eigen::Vector3f temp(0.0f,0.0f,0.0f);
            for (auto itr2 =(*itr)["ai"].begin(); itr2!= (*itr)["ai"].end(); itr2++)
                temp[i++] = (*itr2).get<float>();
            o.setAi(temp);
        }

        if (itr->contains("bkc")) {
            int i = 0;
            Eigen::Vector3f temp(0.0f,0.0f,0.0f);
            for (auto itr2 =(*itr)["bkc"].begin(); itr2!= (*itr)["bkc"].end(); itr2++)
                temp[i++] = (*itr2).get<float>();
            o.setBkc(temp);
        }

        if (itr->contains("raysperpixel")) {
            int i = 0;
            Eigen::Vector3f temp(0.0f,0.0f,0.0f);
            for (auto itr2 =(*itr)["raysperpixel"].begin(); itr2!= (*itr)["raysperpixel"].end(); itr2++)
                temp[i++] = (*itr2).get<unsigned int>();
            o.setRaysperpixel(temp);

            // If 2 values given, use stratified sampling
            if (i == 2)
                o.setSamplingTechnique("stratified");

            // If 3 values given, use stratified grid sampling
            else if (i == 3)
                o.setSamplingTechnique("grid");

            // If 0 or 1 value given, use uniform random sampling (this is default sampling technique)
            else 
                o.setSamplingTechnique("uniform");

        } 
        // If 0 values given for raysperpixel, use uniform random sampling, with 10 samples by default
        else
            o.setRaysperpixel({10,0,0});

        if (itr->contains("antialiasing"))
            o.setAntialiasing((*itr)["antialiasing"].get<bool>());
        else 
            o.setAntialiasing(false);

        if (itr->contains("twosiderender"))
            o.setTwosiderender((*itr)["twosiderender"].get<bool>());
        else 
            o.setTwosiderender(false);

        if (itr->contains("globalillum"))
            o.setGlobalillum((*itr)["globalillum"].get<bool>());
        else 
            o.setGlobalillum(false);

        if (itr->contains("maxbounces"))
            o.setMaxbounces((*itr)["maxbounces"].get<float>());
        else 
            o.setMaxbounces(3);

        if (itr->contains("probterminate"))
            o.setProbterminate((*itr)["probterminate"].get<float>());
        else
            o.setProbterminate(0.33);

        outputs.push_back(new Output(o));
        ++lc;
    }
    cout << "\nWe have: " << lc << " outputs!" << endl;
}