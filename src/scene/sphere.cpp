#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.


  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  // Solving for quadratic equation from ray and sphere intersection
  double a = dot(r.d, r.d);
  double b = 2 * dot(r.o - o, r.d);
  double c = dot(r.o - o, r.o - o) - r2;
  double descriminant = b * b - 4 * a * c;

  // No intersection if descriminant < 0
  if (descriminant < 0) return false;

  double t_large = (-b + sqrt(descriminant)) / (2 * a);
  double t_small = (-b - sqrt(descriminant)) / (2 * a);

  // Check if intersection is within range of ray
  return (t_small >= r.min_t && t_small <= r.max_t) || (t_large >= r.min_t && t_large <= r.max_t);
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  // Solving for quadratic equation from ray and sphere intersection
  double a = dot(r.d, r.d);
  double b = 2 * dot(r.o - o, r.d);
  double c = dot(r.o - o, r.o - o) - r2;
  double descriminant = b * b - 4 * a * c;

  // No intersection if descriminant < 0
  if (descriminant < 0) return false;

  double t_large = (-b + sqrt(descriminant)) / (2 * a);
  double t_small = (-b - sqrt(descriminant)) / (2 * a);

  bool intersect = false;
  // Check if intersection is within range of ray
  if (t_small >= r.min_t && t_small <= r.max_t) {
    intersect = true;

    // When there is a valid intersection, populate Intersection with 
    i->t = t_small;   // t-value of the input ray where the intersection occurs
    i->n = ((r.o + t_small * r.d) - o);   
    i->n.normalize();   // surface normal at the intersection
    i->primitive = this;   // primitive that was intersected 
    i->bsdf = get_bsdf();   // surface material (BSDF) at the hit point
    
    // Update max_t to be the nearest intersection
    r.max_t = t_small;

    
  } 
  else if (t_large >= r.min_t && t_large <= r.max_t) {
    intersect = true;
    
    // When there is a valid intersection, populate Intersection with 
    i->t = t_large;   // t-value of the input ray where the intersection occurs
    i->n = ((r.o + t_large * r.d) - o);   
    i->n.normalize();   // surface normal at the intersection
    i->primitive = this;   // primitive that was intersected 
    i->bsdf = get_bsdf();   // surface material (BSDF) at the hit point
    
    // Update max_t to be the nearest intersection
    r.max_t = t_large;
  }

  return intersect;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
