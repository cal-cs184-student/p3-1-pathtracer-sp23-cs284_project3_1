#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.

  // Great explanation of Moller-Trumbore Algorithm
  // https://stackoverflow.com/questions/42740765/intersection-between-line-and-triangle-in-3d

  Vector3D vector_1_2 = p2 - p1;
  Vector3D vector_1_3 = p3 - p1;
  Vector3D vector_1_o = r.o - p1;
  Vector3D cross_d_1_o = cross(r.d, vector_1_3);
  Vector3D n = cross(vector_1_o, vector_1_2);

  double det = dot(cross_d_1_o, vector_1_2);
  double inverse_det = 1.0/det;

  double t = dot(vector_1_3, n) * inverse_det;
  double beta = dot(vector_1_o, cross_d_1_o) * inverse_det;
  double gamma = dot(r.d, n) * inverse_det;
  double alpha = 1 - beta - gamma;

  return t >= r.min_t && t <= r.max_t && alpha >= 0.0 && alpha <= 1.0 && beta >= 0.0 && beta <= 1.0 && gamma >= 0.0 && gamma <= 1.0;

}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly

  //Moller-Trumbore Algorithm
  Vector3D vector_1_2 = p2 - p1;
  Vector3D vector_1_3 = p3 - p1;
  Vector3D vector_1_o = r.o - p1;
  Vector3D cross_d_1_o = cross(r.d, vector_1_3);
  Vector3D n = cross(vector_1_o, vector_1_2);

  double det = dot(cross_d_1_o, vector_1_2);
  double inverse_det = 1.0/det;

  double t = dot(vector_1_3, n) * inverse_det;
  double beta = dot(vector_1_o, cross_d_1_o) * inverse_det;
  double gamma = dot(r.d, n) * inverse_det;
  double alpha = 1 - beta - gamma;

  bool intersect = t >= r.min_t && t <= r.max_t && alpha >= 0.0 && alpha <= 1.0 && beta >= 0.0 && beta <= 1.0 && gamma >= 0.0 && gamma <= 1.0;
  if (intersect) {

    // When there is a valid intersection, populate Intersection with 
    isect->t = t;   // t-value of the input ray where the intersection occurs
    isect->n = alpha * n1 + beta * n2 + gamma * n3;   // surface normal at the intersection
    isect->primitive = this;   // primitive that was intersected 
    isect->bsdf = get_bsdf(); // surface material (BSDF) at the hit point

    // Update max_t to be the nearest intersection
    // Ignore all future intersections that are farther away than current intersection
    r.max_t = t; 
    
  }
  return intersect;

}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
