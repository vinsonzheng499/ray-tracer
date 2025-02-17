// trimesh.cpp
#include "trimesh.h"
#include <algorithm>
#include <assert.h>
#include <cmath>
#include <float.h>
#include <string.h>
#include "../ui/TraceUI.h"
#include <iostream>
extern TraceUI *traceUI;
extern TraceUI *traceUI;

using namespace std;

Trimesh::Trimesh(Scene *scene, Material *mat, MatrixTransform transform)
    : SceneObject(scene, mat), displayListWithMaterials(0),
      displayListWithoutMaterials(0) {
  this->transform = transform;
  vertNorms = false;
  faceBVH = nullptr;
}

Trimesh::~Trimesh() {
  for (auto f : faces)
    delete f;
  clearFaceBVH();
}

// must add vertices, normals, and materials IN ORDER
void Trimesh::addVertex(const glm::dvec3 &v) { vertices.emplace_back(v); }

void Trimesh::addNormal(const glm::dvec3 &n) { normals.emplace_back(n); }

void Trimesh::addColor(const glm::dvec3 &c) { vertColors.emplace_back(c); }

void Trimesh::addUV(const glm::dvec2 &uv) { uvCoords.emplace_back(uv); }

// Returns false if the vertices a,b,c don't all exist
bool Trimesh::addFace(int a, int b, int c) {
  int vcnt = vertices.size();

  if (a >= vcnt || b >= vcnt || c >= vcnt)
    return false;

  TrimeshFace *newFace = new TrimeshFace(this, a, b, c);
  if (!newFace->degen)
    faces.push_back(newFace);
  else
    delete newFace;

  // Don't add faces to the scene's object list so we can cull by bounding
  // box
  return true;
}

// Check to make sure that if we have per-vertex materials or normals
// they are the right number.
const char *Trimesh::doubleCheck() {
  if (!vertColors.empty() && vertColors.size() != vertices.size())
    return "Bad Trimesh: Wrong number of vertex colors.";
  if (!uvCoords.empty() && uvCoords.size() != vertices.size())
    return "Bad Trimesh: Wrong number of UV coordinates.";
  if (!normals.empty() && normals.size() != vertices.size())
    return "Bad Trimesh: Wrong number of normals.";

  return 0;
}

bool Trimesh::intersectLocal(ray &r, isect &i) const {
  if (faceBVH) {
    return faceBVH->intersect(r, i);
  } else { // If no BVH, do a brute-force intersection
    bool have_one = false;
    for (auto face : faces) {
      isect cur;
        if (face->intersectLocal(r, cur)) {
          if (!have_one || (cur.getT() < i.getT())) {
            i = cur;
            have_one = true;
          }
        }
      }
    if (!have_one)
      i.setT(1000.0);
    return have_one;
  }
}

void Trimesh::buildFaceBVH(int maxDepth, int targetLeafSize) {
  std::lock_guard<std::mutex> vlock(verticesMutex);
  std::lock_guard<std::mutex> flock(facesMutex);
  clearFaceBVH();
  faceBVH = new BVHTree<TrimeshFace>(maxDepth, targetLeafSize);
  std::cout << "Before buildFaceBVH" << std::endl; // Removing causes segfault
  faceBVH->build(faces); // Build from the vector of TrimeshFace*
}

TrimeshFace::TrimeshFace(Trimesh *parent, int a, int b, int c) {
  this->parent = parent;
  ids[0] = a;
  ids[1] = b;
  ids[2] = c;

  // Compute the face normal here, not on the fly
  glm::dvec3 a_coords = parent->vertices[a];
  glm::dvec3 b_coords = parent->vertices[b];
  glm::dvec3 c_coords = parent->vertices[c];

  glm::dvec3 vab = (b_coords - a_coords);
  glm::dvec3 vac = (c_coords - a_coords);
  glm::dvec3 vcb = (b_coords - c_coords);

  if (glm::length(vab) == 0.0 || glm::length(vac) == 0.0 ||
      glm::length(vcb) == 0.0)
    degen = true;
  else {
    degen = false;
    normal = glm::cross(b_coords - a_coords, c_coords - a_coords);
    normal = glm::normalize(normal);
    dist = glm::dot(normal, a_coords);
  }
  localbounds = ComputeLocalBoundingBox();
  bounds = localbounds;
}

BoundingBox TrimeshFace::ComputeLocalBoundingBox() {
  BoundingBox localbounds;
  localbounds.setMax(
      glm::max(parent->vertices[ids[0]], parent->vertices[ids[1]]));
  localbounds.setMin(
      glm::min(parent->vertices[ids[0]], parent->vertices[ids[1]]));

  localbounds.setMax(
      glm::max(parent->vertices[ids[2]], localbounds.getMax()));
  localbounds.setMin(
      glm::min(parent->vertices[ids[2]], localbounds.getMin()));
  return localbounds;
}

bool TrimeshFace::intersect(ray &r, isect &i) const {
  // Transform the ray into the object's local coordinate space
  glm::dvec3 pos = parent->transform.globalToLocalCoords(r.getPosition());
  glm::dvec3 dir =
      parent->transform.globalToLocalCoords(r.getPosition() + r.getDirection()) - pos;
  double length = glm::length(dir);
  dir = glm::normalize(dir);
  // Backup World pos/dir, and switch to local pos/dir
  glm::dvec3 Wpos = r.getPosition();
  glm::dvec3 Wdir = r.getDirection();
  r.setPosition(pos);
  r.setDirection(dir);
  bool rtrn = false;
  if (intersectLocal(r, i)) {
    // Transform the intersection point & normal returned back into
    // global space.
    i.setN(parent->transform.localToGlobalCoordsNormal(i.getN()));
    i.setT(i.getT() / length);
    rtrn = true;
  }
  // Restore World pos/dir
  r.setPosition(Wpos);
  r.setDirection(Wdir);
  return rtrn;
}

// Intersect ray r with the triangle abc.  If it hits returns true,
// and put the parameter in t and the barycentric coordinates of the
// intersection in u (alpha) and v (beta).
bool TrimeshFace::intersectLocal(ray &r, isect &i) const {
  // YOUR CODE HERE
  //
  // FIXME: Add ray-trimesh intersection

  /* To determine the color of an intersection, use the following rules:
     - If the parent mesh has non-empty `uvCoords`, barycentrically interpolate
       the UV coordinates of the three vertices of the face, then assign it to
       the intersection using i.setUVCoordinates().
     - Otherwise, if the parent mesh has non-empty `vertexColors`,
       barycentrically interpolate the colors from the three vertices of the
       face. Create a new material by copying the parent's material, set the
       diffuse color of this material to the interpolated color, and then
       assign this material to the intersection.
     - If neither is true, assign the parent's material to the intersection.
  */

  glm::dvec3 v0 = parent->vertices[ids[0]];
  glm::dvec3 v1 = parent->vertices[ids[1]];
  glm::dvec3 v2 = parent->vertices[ids[2]];

  // Ray Plane Intersection
  double T = glm::dot(v0 - r.getPosition(), normal) / glm::dot(r.getDirection(), normal);

  if (T < 0.0) {
    return false;
  }

  glm::dvec3 p = r.getPosition() + T * r.getDirection();

  glm::dvec3 AB = v1 - v0;
  glm::dvec3 AP = p - v0;
  glm::dvec3 BC = v2 - v1;
  glm::dvec3 BP = p - v1;
  glm::dvec3 CA = v0 - v2;
  glm::dvec3 CP = p - v2;

  // Inside Outside Test
  if (glm::dot(glm::cross(AB, AP), normal) >= 0 &&
      glm::dot(glm::cross(BC, BP), normal) >= 0 &&
      glm::dot(glm::cross(CA, CP), normal) >= 0) {

    i.setT(T);
    i.setN(normal);

    // Barycentric Coordinates
    double areaABC = glm::length(glm::cross(AB, v2 - v0));
    double alpha = glm::length(glm::cross(BP, v2 - v1)) / areaABC;
    double beta  = glm::length(glm::cross(CP, v0 - v2)) / areaABC;
    double gamma = 1.0 - alpha - beta;

    // UV Coordinates
    if (!parent->uvCoords.empty()) {
        glm::dvec2 uvA = parent->uvCoords[ids[0]];
        glm::dvec2 uvB = parent->uvCoords[ids[1]];
        glm::dvec2 uvC = parent->uvCoords[ids[2]];

        glm::dvec2 uv = alpha * uvA + beta * uvB + gamma * uvC;
        i.setUVCoordinates(uv);
    }
    // Vertex Colors
    else if (!parent->vertColors.empty()) {
        glm::dvec3 colorA = parent->vertColors[ids[0]];
        glm::dvec3 colorB = parent->vertColors[ids[1]];
        glm::dvec3 colorC = parent->vertColors[ids[2]];

        glm::dvec3 interpolatedColor = alpha * colorA + beta * colorB + gamma * colorC;

        Material* newMat = new Material(parent->getMaterial());
        newMat->setDiffuse(interpolatedColor);
        i.setMaterial(*newMat);
    }
    else {
      i.setMaterial(parent->getMaterial());
    }

    // Phong Normal Interpolation
    if (!parent->normals.empty()) {
      glm::dvec3 nA = parent->normals[ids[0]];
      glm::dvec3 nB = parent->normals[ids[1]];
      glm::dvec3 nC = parent->normals[ids[2]];

      glm::dvec3 interpolatedNormal = glm::normalize(alpha * nA + beta * nB + gamma * nC);
      i.setN(interpolatedNormal);
    }
    else {
      i.setN(normal); // If no per-vertex normals, use the face normal
    }

    i.setObject(this->parent);
    return true;
  }

  return false;
}

// Once all the verts and faces are loaded, per vertex normals can be
// generated by averaging the normals of the neighboring faces.
void Trimesh::generateNormals() {
  int cnt = vertices.size();
  normals.resize(cnt);
  std::vector<int> numFaces(cnt, 0);

  for (auto face : faces) {
    glm::dvec3 faceNormal = face->getNormal();

    for (int i = 0; i < 3; ++i) {
      normals[(*face)[i]] += faceNormal;
      ++numFaces[(*face)[i]];
    }
  }

  for (int i = 0; i < cnt; ++i) {
    if (numFaces[i])
      normals[i] /= numFaces[i];
  }

  vertNorms = true;
}

BoundingBox Trimesh::ComputeLocalBoundingBox() {
  BoundingBox localbounds;
  if (vertices.size() == 0)
    return localbounds;
  localbounds.setMax(vertices[0]);
  localbounds.setMin(vertices[0]);
  Vertices::const_iterator viter;
  for (viter = vertices.begin(); viter != vertices.end(); ++viter) {
    localbounds.setMax(glm::max(localbounds.getMax(), *viter));
    localbounds.setMin(glm::min(localbounds.getMin(), *viter));
  }
  localBounds = localbounds;
  return localbounds;  
}

void Trimesh::clearFaceBVH() {
  if (faceBVH) {
    delete faceBVH;
    faceBVH = nullptr;
  }
}
