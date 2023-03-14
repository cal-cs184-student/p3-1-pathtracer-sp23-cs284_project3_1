#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.


  BBox bbox;

  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
  }

  BVHNode *node = new BVHNode(bbox);

  int num_primitives = end - start;
  
  // Qualifies as leaf node if number of primitives doesn't exceed max_leaf_size
  if (num_primitives <= max_leaf_size) {
    node->start = start;
    node->end = end;
    node->l = NULL;
    node->r = NULL;
  }

  // Split into left/right internal nodes by spliting along axis
  // Which results in least surface area for left/right bounding boxes
  else {
    Vector3D centroid(0.0);
    for (auto p = start; p != end; p++) {
      centroid += (*p) -> get_bbox().centroid();
    }
    centroid /= (float)num_primitives;

    int axis_min_surface_area = -1;
    float min_surface_area = INFINITY;

    // Find the axis with least surface area
    for (int axis = 0; axis < 3; axis++) {

      vector<Primitive*> left_primitives, right_primitives;
      for (auto p = start; p != end; p++) {
        if ((*p) -> get_bbox().centroid()[axis] > centroid[axis]) {
          right_primitives.push_back(*p);
        }
        else {
          left_primitives.push_back(*p);
        }
      }

      BBox left_bbox, right_bbox;
      for (auto p = left_primitives.begin(); p != left_primitives.end(); p++) {
        BBox primitive_bbox = (*p) -> get_bbox();
        left_bbox.expand(primitive_bbox);
      }

      for (auto p = right_primitives.begin(); p != right_primitives.end(); p++) {
        BBox primitive_bbox = (*p) -> get_bbox();
        right_bbox.expand(primitive_bbox);
      }

      float surface_area = right_primitives.size() * right_bbox.surface_area() + left_primitives.size() * left_bbox.surface_area();

      if (surface_area < min_surface_area) {
        min_surface_area = surface_area;
        axis_min_surface_area = axis;
      }
    }

    vector<Primitive*> left_primitives, right_primitives;
    for (auto p = start; p != end; p++) {
      if ((*p) -> get_bbox().centroid()[axis_min_surface_area] > centroid[axis_min_surface_area]) {
        right_primitives.push_back(*p);
      }
      else {
        left_primitives.push_back(*p);
      }
    }

    
    // Move the primitives into the correct partition
    auto current = start;
    for (int i = 0; i < left_primitives.size(); i++) {
      *current = left_primitives[i];
      current++;
    }
    auto center = current;
    for (int i = 0; i < right_primitives.size(); i++) {
      *current = right_primitives[i];
      current++;
    }

    // Recursively call construct_bvh()
    node -> l = construct_bvh(start, center, max_leaf_size);
    node -> r = construct_bvh(center, end, max_leaf_size); 
  }
  

  return node;


}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  // Ray misses node
  if (!node -> bb.intersect(ray, ray.min_t, ray.max_t)) return false;
   
  // Ray hits internal node
  if (!node -> isLeaf()) {
    return has_intersection(ray, node -> l) || has_intersection(ray, node -> r);
  }

  // Ray hits leaf node
  else {
    for (auto p = node -> start; p != node -> end; p++) {
      total_isects++;
      if ((*p) -> has_intersection(ray)) {
        return true;
      }
    }
  }

  // Ray hits bounding box of leaf node but has no intersection with primitives
  return false;

}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.



  // bool hit = false;
  // for (auto p : primitives) {
  //   total_isects++;
  //   hit = p->intersect(ray, i) || hit;
  // }
  // return hit;
  // total_isects++; // number of interesection tests for the ray
  // if (node->bb.intersect(ray, ray.min_t, ray.max_t)) {
      
  //     // Node is a leaf
  //     if (node->isLeaf()) {
          
  //         bool hit = false;
  //         for (auto p = node->start; p != node->end; p++) {
  //             total_isects++;
  //             bool p_hit = (*p)->intersect(ray, i);
  //             hit = hit || p_hit; 
                            
  //         }
          
  //         return hit;
  //     }
  //     else {
  //         bool hit_left = intersect(ray, i, node->l); 
  //         bool hit_right = intersect(ray, i, node->r);
  //         return hit_left || hit_right;
  //     }
  
  // } else {
  //     return false;
  // }

  // Ray misses node
  if (!node -> bb.intersect(ray, ray.min_t, ray.max_t)) return false;
   
  // Ray hits internal node
  if (!node -> isLeaf()) {
    bool hit_left = intersect(ray, i, node->l); 
    bool hit_right = intersect(ray, i, node->r);
    return hit_left || hit_right;
  }

  // Ray hits leaf node
  else {
    bool hit = false;
    for (auto p = node->start; p != node->end; p++) {
        total_isects++;
        hit = hit || ((*p)->intersect(ray, i));           
    }
    return hit;
  }

  // Ray hits bounding box of leaf node but has no intersection with primitives
  return false;

}

} // namespace SceneObjects
} // namespace CGL
