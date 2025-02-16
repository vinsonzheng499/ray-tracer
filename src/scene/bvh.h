#pragma once

#include "bbox.h"
#include "scene.h"
#include <vector>
#include <algorithm>

class Geometry;

class BVHNode {
public:
    BoundingBox bbox;
    BVHNode* left;
    BVHNode* right;
    std::vector<Geometry*> objects;
    static const int maxObjectsPerLeaf = 4;

    BVHNode() : left(nullptr), right(nullptr) {}
    ~BVHNode() {
        delete left;
        delete right;
    }

    void build(std::vector<Geometry*>& geometries);
    bool intersect(ray& r, isect& i) const;
};
