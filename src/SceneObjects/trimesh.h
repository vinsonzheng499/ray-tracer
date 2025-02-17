// trimesh.h
#ifndef TRIMESH_H__
#define TRIMESH_H__

#include <list>
#include <memory>
#include <vector>

#include "../scene/bvhTree.h"
#include "../scene/material.h"
#include "../scene/ray.h"
#include "../scene/scene.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/vec3.hpp>

class TrimeshFace;

class Trimesh : public SceneObject {
  friend class TrimeshFace;
  typedef std::vector<glm::dvec3> Normals;
  typedef std::vector<glm::dvec3> Vertices;
  typedef std::vector<TrimeshFace *> Faces;
  typedef std::vector<glm::dvec3> VertColors;
  typedef std::vector<glm::dvec2> UVCoords;

  Vertices vertices;
  Faces faces;
  Normals normals;
  VertColors vertColors;
  UVCoords uvCoords;
  BoundingBox localBounds;

public:
  Trimesh(Scene *scene, Material *mat, MatrixTransform transform);
  ~Trimesh() override;

  bool vertNorms;

  bool intersectLocal(ray &r, isect &i) const override;

  // must add vertices, normals, and materials IN ORDER
  void addVertex(const glm::dvec3 &);
  void addNormal(const glm::dvec3 &);
  void addColor(const glm::dvec3 &);
  void addUV(const glm::dvec2 &);
  bool addFace(int a, int b, int c);

  const char *doubleCheck();

  void generateNormals();

  bool hasBoundingBoxCapability() const override { return true; }

  BoundingBox ComputeLocalBoundingBox() override;

  void buildFaceBVH(int maxDepth, int targetLeafSize);
  void clearFaceBVH();

protected:
  void glDrawLocal(int quality, bool actualMaterials,
                   bool actualTextures) const override;
  mutable int displayListWithMaterials;
  mutable int displayListWithoutMaterials;

private:
    BVHTree<TrimeshFace> *faceBVH;
    std::mutex facesMutex;
    std::mutex verticesMutex;
};

/* A triangle in a mesh. This class looks and behaves a lot like other
SceneObjects (e.g. Trimesh, Sphere, etc.) and has many of the same members
like intersectLocal() and a BoundingBox.

However, SceneObjects must have a MatrixTransform and a Material, and storing
these in every single TrimeshFace would explode memory usage. Because of this,
TrimeshFace is treated as an implementation detail of Trimesh and is not within
the SceneObject hierarchy.

Access to materials and transform are provided by referencing the parent
Trimesh object. */
class TrimeshFace {
  Trimesh *parent;
  int ids[3];
  glm::dvec3 normal;
  double dist;
  BoundingBox bounds;

public:
  TrimeshFace(Trimesh *parent, int a, int b, int c);

  BoundingBox localbounds;
  bool degen;

  int operator[](int i) const { return ids[i]; }

  glm::dvec3 getNormal() { return normal; }

  bool intersect(ray &r, isect &i) const;
  bool intersectLocal(ray &r, isect &i) const;
  Trimesh *getParent() const { return parent; }

  bool hasBoundingBoxCapability() const { return true; }

  BoundingBox ComputeLocalBoundingBox();

  const BoundingBox &getBoundingBox() const { return localbounds; }
};

#endif // TRIMESH_H__
