#include "bvh.h"
#include <iostream>

using namespace std;

extern bool debugMode;

void BVHNode::build(std::vector<Geometry*>& geometries, int depth) {
    cout << "depth: " << depth << endl;
    // Initialize node bounds
    if (geometries.empty()) {
        cout << "Do I come here?" << endl;
        return;
    }
    
    bbox = geometries[0]->getBoundingBox();
    for (auto obj : geometries) {
        cout << "for loop" << endl;
        bbox.merge(obj->getBoundingBox());
    }

    // If few enough objects, make this a leaf
    if (geometries.size() <= bvhLeafSize || depth >= bvhMaxDepth) {
        cout << "leaf" << endl;
        objects = geometries;
        return;
    }

    // Find longest axis
    glm::dvec3 extent = bbox.getMax() - bbox.getMin();
    int axis = 0;
    if (extent[1] > extent[0]) axis = 1;
    if (extent[2] > extent[axis]) axis = 2;

    // Sort objects along that axis
    auto comparator = [axis](Geometry* a, Geometry* b) {
        glm::dvec3 centroidA = (a->getBoundingBox().getMin() + 
                               a->getBoundingBox().getMax()) * 0.5;
        glm::dvec3 centroidB = (b->getBoundingBox().getMin() + 
                               b->getBoundingBox().getMax()) * 0.5;
        return centroidA[axis] < centroidB[axis];
    };
    std::sort(geometries.begin(), geometries.end(), comparator);

    // Split into two groups
    size_t mid = geometries.size() / 2;
    std::vector<Geometry*> leftGeom(geometries.begin(), geometries.begin() + mid);
    std::vector<Geometry*> rightGeom(geometries.begin() + mid, geometries.end());

    // Recursively build children
    left = new BVHNode();
    right = new BVHNode();
    left->build(leftGeom, depth + 1);
    right->build(rightGeom, depth + 1);
}

bool BVHNode::intersect(ray& r, isect& i) const {
    double tmin, tmax;
    if (!bbox.intersect(r, tmin, tmax))
        return false;
        
    // Early termination - if we already found something closer
    if (tmin > i.getT())
        return false;

    // Leaf node
    if (!left && !right) {
        bool hit = false;
        for (auto obj : objects) {
            isect temp;
            if (obj->intersect(r, temp) && temp.getT() < i.getT()) {
                i = temp;
                hit = true;
            }
        }
        return hit;
    }

    // Determine traversal order based on ray direction
    BVHNode *first = left, *second = right;
    isect temp = i;  // Save current closest intersection
    bool hit = false;
    
    // Check first child
    if (first && first->intersect(r, i)) {
        hit = true;
        temp = i;  // Update closest hit
    }
    
    // Only check second if it could contain a closer hit
    if (second && second->bbox.intersect(r, tmin, tmax) && tmin < temp.getT()) {
        if (second->intersect(r, i)) {
            hit = true;
        } else {
            i = temp;  // Restore closest hit from first child
        }
    }

    return hit;
}
