#include "Geometry.h"

// Geometry default constructor
Geometry::Geometry() {
    this->type = "";
    this->ac = Eigen::Vector3f(0,0,0);
    this->dc = Eigen::Vector3f(0,0,0);
    this->sc = Eigen::Vector3f(0,0,0);
    this->ka = 0;
    this->kd = 0;
    this->ks = 0;
    this->pc = 0;
    this->t = 0;
    this->visible = true; // default value is true
}

// Geometry copy constructor
Geometry::Geometry(const Geometry &g) {
    this->type = g.type;
    this->ac = g.ac;
    this->dc = g.dc;
    this->sc = g.sc;
    this->ka = g.ka;
    this->kd = g.kd;
    this->ks = g.ks;
    this->pc = g.pc;
    this->t = g.t;
    this->visible = g.visible;
}

// Sphere default constructor
Sphere::Sphere() {
    this->centre = Eigen::Vector3f(0,0,0);
    this->radius = 0;
}

// Sphere copy constructor
Sphere::Sphere(const Sphere &s) : Geometry(s) {
    this->centre = s.centre;
    this->radius = s.radius;
}

// Sphere parameterized constructor
Sphere::Sphere(Eigen::Vector3f centre, float radius) {
    this->centre = centre;
    this->radius = radius;
}

// Rectangle default constructor
Rectangle::Rectangle() {
    this->p1 = Eigen::Vector3f(0,0,0);
    this->p2 = Eigen::Vector3f(0,0,0);
    this->p3 = Eigen::Vector3f(0,0,0);
    this->p4 = Eigen::Vector3f(0,0,0);
    this->normal = Eigen::Vector3f(0,0,0);
}

// Rectangle copy constructor
Rectangle::Rectangle(const Rectangle &r) : Geometry(r) {
    this->p1 = r.p1;
    this->p2 = r.p2;
    this->p3 = r.p3;
    this->p4 = r.p4;
    this->normal = r.normal;
}

// Rectangle parameterized constructor
Rectangle::Rectangle(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, Eigen::Vector3f p4) {
    // A plane is defined as P(A,N) where A = a point on the plane and N = normal vector
    this->p1 = p1;
    this->p2 = p2;
    this->p3 = p3;
    this->p4 = p4;
    this->p4 = p4;

    // Plane's normal is actually calculated in parseGeometry() in Raytracer class
    // this->normal = ((r.p2 - r.p1).cross(r.p4 - r.p1)).normalized();
}

// Sphere's intersects method for checking if a ray intersects the sphere
bool Sphere::intersects(Ray ray) {
    // Formula: ||ray.centre + t*ray.dir - sphere.centre||^2 - sphere.radius^2 = 0
    // Which results in t as a quadratic equation at^2+bt+c=0: 
    float a = (ray.getDir()).dot(ray.getDir()); // a is simply 1 since the dot product of a normalized vector with itself is 1
    float b = 2*(ray.getDir().dot(ray.getCentre() - centre));
    float c = (ray.getCentre() - centre).dot(ray.getCentre() - centre) - radius*radius;

    // Solve for t, considering both possible outcomes t1 and t2 as it's a quadratic equation
    float t1 = (-b+sqrt(b*b-4*a*c))/(2*a);
    float t2 = (-b-sqrt(b*b-4*a*c))/(2*a);

    // To determine if the quadratic equation has solutions, we can use the discriminant
    float discriminant = b*b - 4*a*c;

    // If the discriminant is positive, the quadratic equation has two unique solutions which means the ray intersects the sphere
    // and we have two ray points touching the sphere, in which case we use the closest one on the positive side
    if (discriminant > 0) {
        // If both points are negative, no intersections (we don't want to consider points behind the camera origin)
        if (t1 < 0 && t2 < 0)   
            return false;
        // If one point is positive and the other is negative, we use the positive one
        else if (t1 > 0 && t2 < 0)   
            setT(t1);
        else if (t1 < 0 && t2 > 0)   
            setT(t2);
        // If both points are positive, so in front of the camera origin, choose the one closer to the camera (so the smaller one)
        else if (t1 < t2)   
            setT(t1);
        else   
            setT(t2);
        return true;
    }
            
    // If the discriminant is zero, the quadratic equation has one solution which means the ray is tangent to the sphere. For now, we don't consider this an intersection
    // If the discriminant is negative, the quadratic equation has no real solutions in which case the ray doesn't intersect the sphere
    // In either case we return false
    return false;
}

// Rectangles's intersects method for checking if a ray intersects the plane
bool Rectangle::intersects(Ray ray) {
    // The point of intersection between the ray and the plane exists and is inside the plane if (P-p1.dot(N)) = 0,
    // which expands into the linear equation (o + t*d - p1).dot(N) = 0 where o = ray's origin, t = scalar (what we're solving for), d = ray.dir, N = plane's normal
    float t = ((p1 - ray.getCentre()).dot(normal)) / (ray.getDir().dot(normal));

    // No intersection if t < 0
    if (t < 0)  
        return false;

    setT(t);
                
    // The intersection point
    Eigen::Vector3f p = ray.getCentre() + t*ray.getDir();

    // Now to check if the point is inside the rectangle or not - it's inside if it's on the same side of the half planes
    // Calculate the perpendicular distances from the 4 lines
    float S1 = ((p2 - p1).cross(p - p1)).dot(normal);
    float S2 = ((p3 - p2).cross(p - p2)).dot(normal);
    float S3 = ((p4 - p3).cross(p - p3)).dot(normal);
    float S4 = ((p1 - p4).cross(p - p4)).dot(normal);

    // If S1, S2, S3 and S4 all share the same sign, it means the point is in the rectangle
    if ((S1 > 0 && S2 > 0 && S3 > 0 && S4 > 0) || (S1 < 0 && S2 < 0 && S3 < 0 && S4 < 0))
        return true;

    // In they don't all share the same sign, the point is not in the rectangle
    return false;
}