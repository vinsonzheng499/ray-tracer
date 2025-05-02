// trimesh.cpp
#include "trimesh.h"
#include <algorithm>
#include <assert.h>
#include <cmath>
#include <float.h>
#include <string.h>
#include "../ui/TraceUI.h"
#include <iostream>
#include <limits> // Include for numeric_limits
#include <glm/gtx/norm.hpp> // For length2

// External declaration (assuming TraceUI is defined elsewhere)
extern TraceUI *traceUI;

using namespace std;

// Constructor: Store the passed material as the default/base material (index -1 conceptually)
Trimesh::Trimesh(Scene *scene, Material *mat, MatrixTransform transform)
    : SceneObject(scene, mat), // SceneObject still holds the base material
      displayListWithMaterials(0),
      displayListWithoutMaterials(0) {
  this->transform = transform;
  vertNorms = false;
  faceBVH = nullptr;
  // Ensure the base material is accessible if needed later, though we might not use it for multi-mat OBJs
}

Trimesh::~Trimesh() {
  for (auto f : faces) {
    delete f;
  }
  clearFaceBVH();
}

// must add vertices, normals, and materials IN ORDER
void Trimesh::addVertex(const glm::dvec3 &v) { vertices.emplace_back(v); }

void Trimesh::addNormal(const glm::dvec3 &n) { normals.emplace_back(n); }

void Trimesh::addColor(const glm::dvec3 &c) { vertColors.emplace_back(c); }

void Trimesh::addUV(const glm::dvec2 &uv) { uvCoords.emplace_back(uv); }

// Add a material to the mesh's specific material list
int Trimesh::addMeshMaterial(Material mat, const std::string& name) {
    int index = static_cast<int>(materials.size());
    materials.push_back(mat);
    if (!name.empty()) {
        materialNameMap[name] = index;
    }
    return index;
}

// Get material by index. Return base material if index is -1 or out of bounds.
const Material& Trimesh::getMaterial(int index) const {
    if (index >= 0 && static_cast<size_t>(index) < materials.size()) {
        return materials[index];
    }
    // Fallback to the base material stored in SceneObject
    return SceneObject::getMaterial();
}


// Modified addFace to create TrimeshFace with a material ID
bool Trimesh::addFace(int a, int b, int c, int materialId) {
  int vcnt = static_cast<int>(vertices.size()); // Cast size_t to int for comparison

  if (a >= vcnt || b >= vcnt || c >= vcnt || a < 0 || b < 0 || c < 0) // Added check for negative indices
    return false;

  TrimeshFace *newFace = new TrimeshFace(this, a, b, c, materialId);
  if (!newFace->degen) {
    faces.push_back(newFace);
  } else {
    // Optional: Print a warning for degenerate faces
    // std::cerr << "Warning: Degenerate face skipped (" << a << ", " << b << ", " << c << ")" << std::endl;
    delete newFace;
  }

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

  return nullptr; // Return nullptr on success
}

bool Trimesh::intersectLocal(ray &r, isect &i) const {
  if (faceBVH) {
    return faceBVH->intersect(r, i);
  } else { // If no BVH, do a brute-force intersection
    bool have_one = false;
    double min_t = std::numeric_limits<double>::max(); // Use max double value

    for (auto face : faces) {
      isect cur;
      if (face->intersectLocal(r, cur)) {
        if (!have_one || (cur.getT() < min_t)) {
          i = cur;
          min_t = i.getT(); // Update min_t
          have_one = true;
        }
      }
    }
    if (!have_one) {
      i.setT(min_t); // Set T even if no hit, consistent with old code
    }
    return have_one;
  }
}

void Trimesh::buildFaceBVH(int maxDepth, int targetLeafSize) {
  clearFaceBVH();
  if (!faces.empty()) { // Only build if there are faces
    faceBVH = new BVHTree<TrimeshFace>(maxDepth, targetLeafSize);
    faceBVH->build(faces); // Build from the vector of TrimeshFace*
  }
}

// Constructor for TrimeshFace now includes materialId
TrimeshFace::TrimeshFace(Trimesh *parent, int a, int b, int c, int materialId)
    : transform(parent->transform), materialId(materialId) { // Initialize materialId
  this->parent = parent;
  ids[0] = a;
  ids[1] = b;
  ids[2] = c;

  // Pre-check indices to avoid out-of-bounds access
  size_t vcount = parent->vertices.size();
  if (a < 0 || static_cast<size_t>(a) >= vcount ||
      b < 0 || static_cast<size_t>(b) >= vcount ||
      c < 0 || static_cast<size_t>(c) >= vcount) {
      std::cerr << "Error: Invalid vertex index in TrimeshFace constructor (" << a << ", " << b << ", " << c << ")" << std::endl;
      degen = true; // Mark as degenerate if indices are bad
      localbounds = BoundingBox(); // Initialize bounds
      return; // Stop further processing for this face
  }

  // Compute the face normal here, not on the fly
  glm::dvec3 a_coords = parent->vertices[a];
  glm::dvec3 b_coords = parent->vertices[b];
  glm::dvec3 c_coords = parent->vertices[c];

  glm::dvec3 vab = (b_coords - a_coords);
  glm::dvec3 vac = (c_coords - a_coords);
  // glm::dvec3 vcb = (b_coords - c_coords); // Not needed for normal calculation

  // Check for degenerate edges (very close vertices)
  if (glm::length2(vab) < RAY_EPSILON * RAY_EPSILON ||
      glm::length2(vac) < RAY_EPSILON * RAY_EPSILON ||
      glm::length2(b_coords - c_coords) < RAY_EPSILON * RAY_EPSILON) {
    degen = true;
  } else {
    normal = glm::cross(vab, vac);
    double lenSq = glm::length2(normal);
    if (lenSq > RAY_EPSILON * RAY_EPSILON) { // Check for zero-length normal
        normal /= std::sqrt(lenSq); // Normalize only if length is significant
        degen = false;
        dist = glm::dot(normal, a_coords);
    } else {
        // Handle degenerate triangle case - e.g., set a default normal or mark
        // std::cerr << "Warning: Degenerate normal for face (" << a << ", " << b << ", " << c << ")" << std::endl;
        // We could set a default normal, but marking as degenerate is safer
        degen = true;
    }
  }
  localbounds = ComputeLocalBoundingBox(); // Compute bounds even if degenerate for BVH structure
  bounds = localbounds;
}


BoundingBox TrimeshFace::ComputeLocalBoundingBox() {
  BoundingBox localbounds;

  // Check parent and indices validity before accessing vertices
  if (!parent || ids[0] < 0 || static_cast<size_t>(ids[0]) >= parent->vertices.size() ||
      ids[1] < 0 || static_cast<size_t>(ids[1]) >= parent->vertices.size() ||
      ids[2] < 0 || static_cast<size_t>(ids[2]) >= parent->vertices.size()) {
    localbounds.setEmpty(); // Return empty box if indices are invalid
    return localbounds;
  }

  glm::dvec3 v0 = parent->vertices[ids[0]];
  glm::dvec3 v1 = parent->vertices[ids[1]];
  glm::dvec3 v2 = parent->vertices[ids[2]];

  localbounds.setMin(glm::min(glm::min(v0, v1), v2));
  localbounds.setMax(glm::max(glm::max(v0, v1), v2));

  return localbounds;
}

bool TrimeshFace::intersect(ray &r, isect &i) const {
  return intersectLocal(r, i);
}

// Intersect ray r with the triangle abc. If it hits returns true,
// and put the parameter in t and the barycentric coordinates of the
// intersection in u (alpha) and v (beta).
bool TrimeshFace::intersectLocal(ray &r, isect &i) const {
  // If the face is degenerate, skip intersection
  if (degen) {
      return false;
  }

  // Check parent and indices validity before accessing vertices
  if (!parent || ids[0] < 0 || static_cast<size_t>(ids[0]) >= parent->vertices.size() ||
      ids[1] < 0 || static_cast<size_t>(ids[1]) >= parent->vertices.size() ||
      ids[2] < 0 || static_cast<size_t>(ids[2]) >= parent->vertices.size()) {
    return false; // Cannot intersect if indices are invalid
  }


  glm::dvec3 v0 = parent->vertices[ids[0]];
  glm::dvec3 v1 = parent->vertices[ids[1]];
  glm::dvec3 v2 = parent->vertices[ids[2]];

  // Ray Plane Intersection
  double NdotD = glm::dot(r.getDirection(), normal);

  // Check if ray is parallel to the triangle plane or points away
  if (std::abs(NdotD) < RAY_EPSILON) { // Ray is parallel or nearly parallel
    return false;
  }

  double T = (dist - glm::dot(r.getPosition(), normal)) / NdotD;

  // Check if intersection is behind the ray origin or too far
  if (T < RAY_EPSILON) {
    return false;
  }

  glm::dvec3 p = r.at(T); // Intersection point

  // Inside Outside Test using Barycentric Coordinates directly might be more stable
  glm::dvec3 edge0 = v1 - v0;
  glm::dvec3 edge1 = v2 - v1;
  glm::dvec3 edge2 = v0 - v2;
  glm::dvec3 vp0 = p - v0;
  glm::dvec3 vp1 = p - v1;
  glm::dvec3 vp2 = p - v2;

  // Calculate dot products for barycentric check. Note: Normal direction matters.
  // Assuming counter-clockwise vertex order from outside view.
  // Use a small tolerance for floating-point comparisons.
  if (glm::dot(normal, glm::cross(edge0, vp0)) < -RAY_EPSILON ||
      glm::dot(normal, glm::cross(edge1, vp1)) < -RAY_EPSILON ||
      glm::dot(normal, glm::cross(edge2, vp2)) < -RAY_EPSILON) {
    return false; // Point is outside the triangle
  }

  // If we reach here, the intersection is valid
  i.setT(T);
  i.setN(normal); // Use face normal initially

  // Barycentric Coordinates Calculation (Area method)
  double areaABC = glm::length(glm::cross(edge0, v2 - v0)); // Area of the main triangle * 2
  // Prevent division by zero for degenerate triangles (area should be > epsilon)
  if (areaABC < RAY_EPSILON) return false;

  // Calculate areas of sub-triangles. Use the length of the cross product, which is 2*Area.
  double areaPBC = glm::length(glm::cross(v1 - p, v2 - p));
  double areaPCA = glm::length(glm::cross(v2 - p, v0 - p));

  double alpha = areaPBC / areaABC; // Weight for v0
  double beta  = areaPCA / areaABC;  // Weight for v1
  double gamma = 1.0 - alpha - beta; // Weight for v2

  // Clamp barycentric coordinates to avoid floating point issues at edges/vertices
  alpha = glm::clamp(alpha, 0.0, 1.0);
  beta = glm::clamp(beta, 0.0, 1.0);
  gamma = glm::clamp(gamma, 0.0, 1.0);

  // Renormalize slightly if sum isn't exactly 1 due to precision issues
  double sum = alpha + beta + gamma;
  if (std::abs(sum - 1.0) > RAY_EPSILON && sum > RAY_EPSILON) {
      alpha /= sum;
      beta /= sum;
      gamma /= sum;
  }

  i.setBary(alpha, beta, gamma); // Store barycentric coordinates


    // UV Coordinates
    if (!parent->uvCoords.empty()) {
        // Ensure UV coords exist for all vertices of this face
        size_t uvCount = parent->uvCoords.size();
        if (static_cast<size_t>(ids[0]) < uvCount && static_cast<size_t>(ids[1]) < uvCount && static_cast<size_t>(ids[2]) < uvCount) {
            glm::dvec2 uvA = parent->uvCoords[ids[0]];
            glm::dvec2 uvB = parent->uvCoords[ids[1]];
            glm::dvec2 uvC = parent->uvCoords[ids[2]];
            glm::dvec2 uv = alpha * uvA + beta * uvB + gamma * uvC;
            i.setUVCoordinates(uv);
        } else {
             // Handle error or default UVs if indices are out of bounds
             // std::cerr << "Warning: UV index out of bounds for face vertices (" << ids[0] << "," << ids[1] << "," << ids[2] << ")" << std::endl;
             i.setUVCoordinates(glm::dvec2(0.0, 0.0)); // Default UV
        }
    }
    // Vertex Colors - Only apply if no UVs are present (standard practice)
    else if (!parent->vertColors.empty()) {
         // Ensure vertex colors exist for all vertices of this face
        size_t vcCount = parent->vertColors.size();
        if (static_cast<size_t>(ids[0]) < vcCount && static_cast<size_t>(ids[1]) < vcCount && static_cast<size_t>(ids[2]) < vcCount) {
            glm::dvec3 colorA = parent->vertColors[ids[0]];
            glm::dvec3 colorB = parent->vertColors[ids[1]];
            glm::dvec3 colorC = parent->vertColors[ids[2]];
            glm::dvec3 interpolatedColor = alpha * colorA + beta * colorB + gamma * colorC;

            Material newMat = parent->getMaterial(materialId); // Get the correct material first
            newMat.setDiffuse(interpolatedColor); // Set its diffuse color
            i.setMaterial(newMat); // Assign the modified material
        } else {
             // Handle error or default material if indices are out of bounds
            //  std::cerr << "Warning: Vertex color index out of bounds for face vertices (" << ids[0] << "," << ids[1] << "," << ids[2] << ")" << std::endl;
             i.setMaterial(parent->getMaterial(materialId)); // Use face material without vertex color
        }
    }
    else {
      // Use the material specified by the face's materialId
      i.setMaterial(parent->getMaterial(materialId));
    }

  // Phong Normal Interpolation (if vertex normals are available)
  if (!parent->normals.empty() && parent->vertNorms) {
      // Ensure normals exist for all vertices of this face
      size_t normCount = parent->normals.size();
      if (static_cast<size_t>(ids[0]) < normCount && static_cast<size_t>(ids[1]) < normCount && static_cast<size_t>(ids[2]) < normCount) {
          glm::dvec3 nA = parent->normals[ids[0]];
          glm::dvec3 nB = parent->normals[ids[1]];
          glm::dvec3 nC = parent->normals[ids[2]];
          // Interpolate and normalize
          glm::dvec3 interpolatedNormal = alpha * nA + beta * nB + gamma * nC;
          double lenSq = glm::length2(interpolatedNormal);
          if (lenSq > RAY_EPSILON * RAY_EPSILON) {
            i.setN(interpolatedNormal / std::sqrt(lenSq));
          } else {
            i.setN(normal); // Fallback to face normal if interpolated normal is zero length
          }
      } else {
          // Handle error or use face normal if indices are out of bounds
          // std::cerr << "Warning: Normal index out of bounds for face vertices (" << ids[0] << "," << ids[1] << "," << ids[2] << ")" << std::endl;
          i.setN(normal);
      }
  } else {
    i.setN(normal); // Use face normal if no per-vertex normals
  }

  i.setObject(this->parent); // Intersection is with the parent Trimesh
  return true;
}


// Once all the verts and faces are loaded, per vertex normals can be
// generated by averaging the normals of the neighboring faces.
void Trimesh::generateNormals() {
  int cnt = static_cast<int>(vertices.size());
  if (cnt == 0) return; // Avoid division by zero if no vertices

  normals.assign(cnt, glm::dvec3(0.0)); // Initialize normals to zero, ensures correct size
  std::vector<int> numFaces(cnt, 0);

  for (auto* face : faces) { // Iterate using const pointer
    // Skip degenerate faces in normal generation
    if (face->degen) continue;

    glm::dvec3 faceNormal = face->getNormal();

    for (int i = 0; i < 3; ++i) {
       int vertIndex = (*face)[i]; // Get vertex index
      // Ensure vertex index is valid before accessing normals and numFaces
       if (vertIndex >= 0 && vertIndex < cnt) {
            normals[vertIndex] += faceNormal;
            ++numFaces[vertIndex];
       } else {
            // Handle or log invalid vertex index in face data if necessary
            std::cerr << "Warning: Invalid vertex index " << vertIndex << " encountered during normal generation." << std::endl;
       }
    }
  }

  for (int i = 0; i < cnt; ++i) {
    if (numFaces[i] > 0) {
        // Normalize the summed normals only if the sum is non-zero
        double lenSq = glm::length2(normals[i]);
        if (lenSq > RAY_EPSILON * RAY_EPSILON) {
            normals[i] /= std::sqrt(lenSq);
        } else {
            // Handle cases where summed normal is zero (e.g., two opposite faces)
            // Maybe set to a default normal or handle based on geometry
            normals[i] = glm::dvec3(0.0, 0.0, 1.0); // Example: default to Z-up
        }
    }
    // else: keep normal as zero or set to a default if preferred
  }

  vertNorms = true;
}


BoundingBox Trimesh::ComputeLocalBoundingBox() {
  BoundingBox localbounds;
  if (vertices.empty()) { // Check if vertices vector is empty
    localbounds.setEmpty();
    return localbounds;
  }
  localbounds.setMin(vertices[0]); // Initialize min/max with the first vertex
  localbounds.setMax(vertices[0]);
  // Use iterators correctly
  for (auto viter = vertices.begin() + 1; viter != vertices.end(); ++viter) {
    localbounds.setMax(glm::max(localbounds.getMax(), *viter));
    localbounds.setMin(glm::min(localbounds.getMin(), *viter));
  }
  localBounds = localbounds; // Update the member variable as well
  return localbounds;
}


void Trimesh::clearFaceBVH() {
  if (faceBVH) {
    delete faceBVH;
    faceBVH = nullptr;
  }
}
