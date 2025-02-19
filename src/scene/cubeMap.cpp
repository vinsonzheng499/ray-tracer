#include "cubeMap.h"
#include "../scene/material.h"
#include "../ui/TraceUI.h"
#include "ray.h"
extern TraceUI *traceUI;

glm::dvec3 CubeMap::getColor(ray r) const {
  // YOUR CODE HERE
  // FIXME: Implement Cube Map here

  // Get the normalized direction vector of the ray
  glm::dvec3 dir = glm::normalize(r.getDirection());
  
  // Find the largest magnitude component to determine which face to use
  double absX = std::abs(dir.x);
  double absY = std::abs(dir.y);
  double absZ = std::abs(dir.z);
  
  double maxAxis, u, v;
  int faceIndex;
  
  // Right face: +x
  if (absX >= absY && absX >= absZ && dir.x > 0) {
    faceIndex = 0;  // xpos
    maxAxis = absX;
    u = -dir.z;
    v = -dir.y;
  }
  // Left face: -x
  else if (absX >= absY && absX >= absZ && dir.x <= 0) {
    faceIndex = 1;  // xneg
    maxAxis = absX;
    u = dir.z;
    v = -dir.y;
  }
  // Top face: +y
  else if (absY >= absX && absY >= absZ && dir.y > 0) {
    faceIndex = 2;  // ypos
    maxAxis = absY;
    u = dir.x;
    v = dir.z;
  }
  // Bottom face: -y
  else if (absY >= absX && absY >= absZ && dir.y <= 0) {
    faceIndex = 3;  // yneg
    maxAxis = absY;
    u = dir.x;
    v = -dir.z;
  }
  // Front face: +z (looking from +z direction)
  else if (dir.z > 0) {
    faceIndex = 5;  // zpos
    maxAxis = absZ;
    u = dir.x;
    v = -dir.y;
  }
  // Back face: -z
  else {
    faceIndex = 4;  // zneg
    maxAxis = absZ;
    u = dir.x;
    v = dir.y;
  }
  
  // Check if we have a texture map for this face
  if (!tMap[faceIndex]) {
    return glm::dvec3(0.0, 0.0, 0.0);
  }
  
  // Convert to [0,1] range for texture lookup
  glm::dvec2 texCoord((u/maxAxis + 1.0) * 0.5, (v/maxAxis + 1.0) * 0.5);
  
  // Sample the texture using the calculated coordinates
  return tMap[faceIndex]->getMappedValue(texCoord);
}

CubeMap::CubeMap() {}

CubeMap::~CubeMap() {}

void CubeMap::setNthMap(int n, TextureMap *m) {
  if (m != tMap[n].get())
    tMap[n].reset(m);
}
