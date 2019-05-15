#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CGL { namespace StaticScene {


bool Sphere::test(const Ray& r, double& t1, double& t2) const {
  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  
  double a = dot(r.d, r.d);
  double b = 2 * dot((r.o - o), r.d);
  double c = dot((r.o - o), (r.o - o)) - r2;
  double delta = b * b - 4 * a * c;
  if (delta >=0)
  {
    double a1 = (-b - sqrt(delta)) / (2 * a);
    double a2 = (-b + sqrt(delta)) / (2 * a);
    t1 = min(a1, a2);
    t2 = max(a1, a2);
    // both t1 and t2 are valid
    if(a1>=r.min_t && a1<=r.max_t && a2>=r.min_t && a2<=r.max_t)
    {
      return true;
    }
      // only t1 is valid
    else if(a1>=r.min_t && a1<=r.max_t && (a2<r.min_t || a2>r.max_t))
    {
      return true;
    }
      // only t2 is valid
    if(a2>=r.min_t && a2<=r.max_t && (a1<r.min_t || a1>r.max_t))
    {
      swap(t1, t2);
      return true;
    }
    // both t1 and t2 are invalid
    return false;
  }
  return false;
}

bool Sphere::intersect(const Ray& r) const {
  
  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1, t2;
  if(test(r, t1, t2))
  {
    r.max_t = t1;
    return true;
  }
  
  return false;
  
}

bool Sphere::intersect(const Ray& r, Intersection *i) const{
  
  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double t1, t2;
  if(test(r, t1, t2))
  {
    r.max_t = t1;
    Vector3D N(r.o + t1 * r.d - o);
    N.normalize();
    
    i->t = t1;
    i->primitive = this;
    i->n = N;
    i->bsdf = get_bsdf();
    
    return true;
  }
  return false;
}

void Sphere::draw(const Color& c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c, float alpha) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CGL
