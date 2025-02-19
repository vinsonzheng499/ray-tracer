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
    if (objects.size() <= 1) return 0;
    
    // Calculate centroids and find the split position
    std::vector<double> centroids;
    centroids.reserve(objects.size());
    for (const auto& obj : objects) {
        centroids.push_back(obj->getBoundingBox().centroid()[axis]);
    }
    
    // Find true median value
    size_t mid = centroids.size() / 2;
    std::nth_element(centroids.begin(), centroids.begin() + mid, centroids.end());
    double median = centroids[mid];
    
    // Add small epsilon to handle coincident centroids
    const double epsilon = 1e-8;
    
    // Partition objects
    auto it = std::partition(objects.begin(), objects.end(),
        [axis, median, epsilon](const Obj* obj) {
            double centroid = obj->getBoundingBox().centroid()[axis];
            if (std::abs(centroid - median) < epsilon) {
                // For coincident centroids, alternate between left and right
                static bool goLeft = true;
                goLeft = !goLeft;
                return goLeft;
            }
            return centroid < median;
        });
    
    // Ensure we don't create empty partitions
    int splitIndex = std::distance(objects.begin(), it);
    if (splitIndex == 0) return 1;
    if (splitIndex == objects.size()) return objects.size() - 1;
    
    return splitIndex;
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

    // For leaf nodes
    if (!node->objects.empty()) {
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
    
    // For internal nodes, traverse children
    bool hitLeft = false;
    bool hitRight = false;
    isect leftIsect, rightIsect;
    
    // Initialize with current best intersection
    leftIsect = rightIsect = i;
    
    if (node->left) {
        hitLeft = intersectRecursive(node->left.get(), r, leftIsect);
    }
    if (node->right) {
        hitRight = intersectRecursive(node->right.get(), r, rightIsect);
    }
    
    // Update intersection with closest hit
    if (hitLeft && hitRight) {
        i = (leftIsect.getT() < rightIsect.getT()) ? leftIsect : rightIsect;
    } else if (hitLeft) {
        i = leftIsect;
    } else if (hitRight) {
        i = rightIsect;
    }
    
    return hitLeft || hitRight;
}
