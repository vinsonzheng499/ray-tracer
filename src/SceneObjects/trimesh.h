// trimesh.h
#ifndef TRIMESH_H__
#define TRIMESH_H__

#include <list>
#include <memory>
#include <vector>
#include <map> // Include map for material name lookup

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

  // Store multiple materials for OBJ loading
  std::vector<Material> materials;
  // Map material names to indices in the materials vector (optional but useful)
  std::map<std::string, int> materialNameMap;

public:
  // Constructor now takes a default material, used if no others are assigned
  // or for non-OBJ trimeshes.
  Trimesh(Scene *scene, Material *mat, MatrixTransform transform);
  ~Trimesh() override;

  bool vertNorms;

  bool intersectLocal(ray &r, isect &i) const override;

  // Functions to add mesh data
  void addVertex(const glm::dvec3 &);
  void addNormal(const glm::dvec3 &);
  void addColor(const glm::dvec3 &);
  void addUV(const glm::dvec2 &);
  // Modified addFace to take a material index
  bool addFace(int a, int b, int c, int materialId = -1); // Default to -1 (use base material)

  // Add a material to the mesh's list and return its index
  int addMeshMaterial(Material mat, const std::string& name = "");
  // Get material by index
  const Material& getMaterial(int index = -1) const;

  const char *doubleCheck();

  void generateNormals();

  bool hasBoundingBoxCapability() const override { return true; }

  BoundingBox ComputeLocalBoundingBox() override;

  void buildFaceBVH(int maxDepth, int targetLeafSize);
  void clearFaceBVH();

  // Method to get the number of mesh-specific materials
  size_t getNumMaterials() const { return materials.size(); }

protected:
  void glDrawLocal(int quality, bool actualMaterials,
                   bool actualTextures) const override;
  // Display lists might need adjustment for multiple materials
  // For simplicity, we'll skip using them if multiple materials exist for now
  mutable int displayListWithMaterials;
  mutable int displayListWithoutMaterials;

private:
    BVHTree<TrimeshFace> *faceBVH;
};

/* A triangle in a mesh. */
class TrimeshFace {
  Trimesh *parent;
  int ids[3];
  glm::dvec3 normal;
  double dist;
  BoundingBox bounds;
  int materialId; // Index into the parent Trimesh's materials vector

public:
  // Constructor now takes materialId
  TrimeshFace(Trimesh *parent, int a, int b, int c, int materialId = -1);

  BoundingBox localbounds;
  bool degen;

  int operator[](int i) const { return ids[i]; }

  glm::dvec3 getNormal() { return normal; }
  int getMaterialId() const { return materialId; } // Getter for material ID

  bool intersect(ray &r, isect &i) const;
  bool intersectLocal(ray &r, isect &i) const;
  Trimesh *getParent() const { return parent; }

  bool hasBoundingBoxCapability() const { return true; }

  BoundingBox ComputeLocalBoundingBox();

  const BoundingBox &getBoundingBox() const { return localbounds; }

private:
  MatrixTransform transform;
};

#endif // TRIMESH_H__
