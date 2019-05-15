#include "bvh.h"

#include "CGL/CGL.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL { namespace StaticScene {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  root = construct_bvh(_primitives, max_leaf_size);

}

BVHAccel::~BVHAccel() {
  if (root) delete root;
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

void BVHAccel::draw(BVHNode *node, const Color& c, float alpha) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->draw(c, alpha);
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color& c, float alpha) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->drawOutline(c, alpha);
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(const std::vector<Primitive*>& prims, size_t max_leaf_size) {
  
  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox centroid_box, bbox;

  BVHNode *node = new BVHNode(bbox);

  for (Primitive* p : prims)
  {
    BBox bb = p -> get_bbox();
    bbox.expand(bb);
    Vector3D central = bbox.centroid();
    centroid_box.expand(central);
  }
  
  node -> bb = bbox;
  
  if (prims.size() <= max_leaf_size)
  {
    node->prims = new vector<Primitive *>(prims);
    
    return node;
  }
  
  int axis;
  
  double max_extent = max(centroid_box.extent.x, max(centroid_box.extent.y, centroid_box.extent.z));
  if (max_extent == centroid_box.extent.x)
    axis = 0;
  else if (max_extent == centroid_box.extent.y)
    axis = 1;
  else
    axis = 2;
  
  double splitPoint = centroid_box.centroid()[axis];
  
  vector<Primitive*>left;
  vector<Primitive*>right;
  
  for (Primitive *p : prims)
  {
    if (p -> get_bbox().centroid()[axis] < splitPoint)
      left.push_back(p);
    else
      right.push_back(p);
  }
  
  double right_boundary = bbox.max[axis];
  double left_boundary = bbox.min[axis];
  
  while (left.empty() || right.empty())
  {
    if (left.empty() && !right.empty())
    {
      left_boundary = splitPoint;
      splitPoint = (splitPoint + right_boundary) / 2.0;
    }
    else if (!left.empty() && right.empty())
    {
      right_boundary = splitPoint;
      splitPoint = (left_boundary + splitPoint) / 2.0;
    }
    
    left.clear();
    right.clear();
  
    for (Primitive *p : prims)
    {
      total_isects++;
      if (p -> get_bbox().centroid()[axis] < splitPoint)
        left.push_back(p);
      else
        right.push_back(p);
    }
  }
  
  node -> l = construct_bvh(left, max_leaf_size);
  node -> r = construct_bvh(right, max_leaf_size);
  
  return node;
  
}


bool BVHAccel::intersect(const Ray& ray, BVHNode *node) const {

  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
  
  double t0, t1;
  if (!node->bb.intersect(ray, t0, t1))
  {
    return false;
  }
  else
  {
    // Only both two intersection points are out of the range will cause false

    if (t1 < ray.min_t || t0 > ray.max_t)
    {
      return false;
    }
    else
    {
      if (node->isLeaf())
      {
        bool hit = false;
        for (Primitive *p : *(node->prims))
        {
          total_isects++;
          if (p->intersect(ray))
          {
            hit = true;
          }
        }
        return hit;
      }
      else
      {
        bool left = false;
        bool right = false;
        left = left || intersect(ray, node->l);
        right = right || intersect(ray, node->r);
        return left || right;
      }
    }
  }
}

bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode *node) const {

  // TODO (Part 2.3):
  // Fill in the intersect function.
  double t0, t1;
  if (!node->bb.intersect(ray, t0, t1))
  {
    return false;
  }
  else
  {
    if (t1 < ray.min_t || t0 > ray.max_t)
    {
      return false;
    }
    
    else
    {
      if (node->isLeaf())
      {
        bool hit = false;
        for (Primitive *p : *(node->prims))
        {
          total_isects++;
          if (p->intersect(ray, i))
          {
            hit = true;
          }
        }
        return hit;
      }
      else
      {
        bool left = false;
        bool right = false;
        left = left || intersect(ray, i, node->l);
        right = right || intersect(ray, i, node->r);
        return left || right;
      }
    }
  }
}

}  // namespace StaticScene
}  // namespace CGL
