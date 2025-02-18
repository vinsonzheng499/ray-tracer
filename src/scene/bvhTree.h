// bvhTree.h

#pragma once

#include "bbox.h"
#include "scene.h"
#include <algorithm>
#include <memory>  // Include for unique_ptr
#include <vector>
#include <iostream>

template <typename Obj> class BVHTree {
public:
  BVHTree() : maxDepth(10), targetLeafSize(5) {}
  BVHTree(int maxDepth, int targetLeafSize)
      : maxDepth(maxDepth), targetLeafSize(targetLeafSize) {}
  ~BVHTree();

  void build(std::vector<Obj *> &objects);
  bool intersect(ray &r, isect &i) const;

private:
  struct BVHNode {
    BoundingBox bbox;
    std::unique_ptr<BVHNode> left;
    std::unique_ptr<BVHNode> right;
    std::vector<Obj *> objects;

    BVHNode() = default; // Use default constructor
    ~BVHNode() = default; // Let unique_ptr handle deletion
  };

  std::unique_ptr<BVHNode> root; // Use unique_ptr for root
  int maxDepth;
  int targetLeafSize;

  std::unique_ptr<BVHNode> buildRecursive(std::vector<Obj *> &objects,
                                             int depth);
  bool intersectRecursive(BVHNode *node, ray &r, isect &i) const;

  int partitionObjects(std::vector<Obj *> &objects, int axis);
};

template <typename Obj> BVHTree<Obj>::~BVHTree() = default; // Use default destructor

template <typename Obj>
void BVHTree<Obj>::build(std::vector<Obj *> &objects) {
  root = buildRecursive(objects, 0);
}

template <typename Obj>
std::unique_ptr<typename BVHTree<Obj>::BVHNode>
BVHTree<Obj>::buildRecursive(std::vector<Obj *> &objects, int depth) {
  std::unique_ptr<BVHNode> node = std::make_unique<BVHNode>(); // Use make_unique

  node->objects = objects;

  // Calculate bounding box for the objects in this node
  BoundingBox bbox;
  for (Obj *obj : objects) {
    bbox.merge(obj->getBoundingBox());
  }
  node->bbox = bbox;

  // Base case: If the number of objects is small enough or the maximum depth
  // is reached, create a leaf node
  if (objects.size() <= targetLeafSize || depth >= maxDepth) {
    return node;
  }

  // Recursive case: Partition the objects and create child nodes
  int axis = bbox.longestAxis();
  int splitIndex = partitionObjects(objects, axis);

  // If the partitioning didn't split the objects (e.g., all objects are at
  // the same location), create a leaf node
  if (splitIndex == 0 || splitIndex == objects.size()) {
    return node;
  }

  std::vector<Obj *> leftObjects(objects.begin(),
                                 objects.begin() + splitIndex);
  std::vector<Obj *> rightObjects(objects.begin() + splitIndex, objects.end());

  node->left = buildRecursive(leftObjects, depth + 1);
  node->right = buildRecursive(rightObjects, depth + 1);
  node->objects.clear(); // No longer needed in this node

  return node;
}

template <typename Obj>
int BVHTree<Obj>::partitionObjects(std::vector<Obj *> &objects, int axis) {
  // Sort objects along the chosen axis using a lambda for comparison
  std::sort(objects.begin(), objects.end(),
            [axis](const Obj *a, const Obj *b) {
              glm::dvec3 centroidA = a->getBoundingBox().centroid();
              glm::dvec3 centroidB = b->getBoundingBox().centroid();
              return centroidA[axis] < centroidB[axis];
            });

  // Return the midpoint index as the split point
  return objects.size() / 2;
}

template <typename Obj>
bool BVHTree<Obj>::intersect(ray &r, isect &i) const {
  if (!root)
    return false;
  return intersectRecursive(root.get(), r, i);
}

template <typename Obj>
bool BVHTree<Obj>::intersectRecursive(BVHNode *node, ray &r, isect &i) const {
  double tmin, tmax;
  if (!node->bbox.intersect(r, tmin, tmax)) {
    return false;
  }

  if (node->objects.empty()) {
    // This is an intermediate node, so recursively check its children
    bool hitLeft = false;
    bool hitRight = false;

    if (node->left) {
      hitLeft = intersectRecursive(node->left.get(), r, i);
    }

    if (node->right) {
      hitRight = intersectRecursive(node->right.get(), r, i);
    }

    return hitLeft || hitRight;
  } else {
    // This is a leaf node, so check for intersections with the objects
    bool have_one = false;
    for (const auto &obj : node->objects) {
      isect cur;
      if (obj->intersect(r, cur)) {
        if (!have_one || (cur.getT() < i.getT())) {
          i = cur;
          have_one = true;
        }
      }
    }
    return have_one;
  }
}
