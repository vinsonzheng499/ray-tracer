#include "material.h"
#include "../ui/TraceUI.h"
#include "light.h"
#include "ray.h"
extern TraceUI *traceUI;

#include "../fileio/images.h"
#include <glm/gtx/io.hpp>
#include <iostream>

using namespace std;
extern bool debugMode;

Material::~Material() {}

// Apply the phong model to this point on the surface of the object, returning
// the color of that point.
glm::dvec3 Material::shade(Scene *scene, const ray &r, const isect &i) const {
  // YOUR CODE HERE

  // For now, this method just returns the diffuse color of the object.
  // This gives a single matte color for every distinct surface in the
  // scene, and that's it.  Simple, but enough to get you started.
  // (It's also inconsistent with the phong model...)

  // Your mission is to fill in this method with the rest of the phong
  // shading model, including the contributions of all the light sources.
  // You will need to call both distanceAttenuation() and
  // shadowAttenuation()
  // somewhere in your code in order to compute shadows and light falloff.
  //	if( debugMode )
  //		std::cout << "Debugging Phong code..." << std::endl;

  // When you're iterating through the lights,
  // you'll want to use code that looks something
  // like this:
  //
  
  glm::dvec3 N = glm::normalize(i.getN()); // surface normal at shading point
  glm::dvec3 V = glm::normalize(-r.getDirection()); // view direction from surface to camera
  if (glm::dot(N, V) < 0.0) {
    N = -N;
  }
  glm::dvec3 I_a = scene->ambient(); // ambient light intensity
  glm::dvec3 ambient = ka(i) * I_a; // ambient term
  glm::dvec3 I_phong = ke(i) + ambient; // final intensity at surface
  glm::dvec3 P = r.at(i.getT()); // position of intersection along ray

  for ( const auto& pLight : scene->getAllLights() )
  {
    glm::dvec3 I_l = pLight->getColor(); // intensity of light source
    glm::dvec3 L = glm::normalize(pLight->getDirection(P)); // Direction vector from surface point to light source
    double N_dot_L = glm::dot(i.getN(), L);
    glm::dvec3 diffuse = kd(i) * N_dot_L; // diffuse term
    if (Trans()) {
      diffuse = glm::abs(diffuse);
    } else {
      diffuse = glm::clamp(diffuse, 0.0, 1.0);
    }
    glm::dvec3 R = glm::reflect(-L, i.getN()); // reflection vector of light direction about surface normal
    double V_dot_R = max(glm::dot(glm::normalize(-r.getDirection()), R), 0.0); // clamped dot product
    double ns = shininess(i); // shininess exponent
    glm::dvec3 specular = ks(i) * pow(V_dot_R, ns); // specular term

    double distanceAttenuation = pLight->distanceAttenuation(P);
    glm::dvec3 shadowAttenuation = pLight->shadowAttenuation(r, P);

    I_phong += I_l * (diffuse + specular) * distanceAttenuation * shadowAttenuation;
  }
  return glm::clamp(I_phong, 0.0, 1.0);
}

TextureMap::TextureMap(string filename) {
  data = readImage(filename.c_str(), width, height);
  if (data.empty()) {
    width = 0;
    height = 0;
    string error("Unable to load texture map '");
    error.append(filename);
    error.append("'.");
    throw TextureMapException(error);
  }
}

glm::dvec3 TextureMap::getMappedValue(const glm::dvec2 &coord) const {
  // YOUR CODE HERE
  //
  // In order to add texture mapping support to the
  // raytracer, you need to implement this function.
  // What this function should do is convert from
  // parametric space which is the unit square
  // [0, 1] x [0, 1] in 2-space to bitmap coordinates,
  // and use these to perform bilinear interpolation
  // of the values.

  // Convert from UV coordinates [0,1] to texture space coordinates
  double u = coord.x * (width - 1);
  double v = coord.y * (height - 1);
  
  // Get the four nearest texel coordinates
  int u1 = static_cast<int>(floor(u));
  int v1 = static_cast<int>(floor(v));
  int u2 = u1 + 1;
  int v2 = v1 + 1;
  
  // Calculate α and β
  double alpha = u - u1;
  double beta = v - v1;
  
  // Get the four corner colors
  glm::dvec3 c11 = getPixelAt(u1, v1); // Bottom-left
  glm::dvec3 c21 = getPixelAt(u2, v1); // Bottom-right
  glm::dvec3 c12 = getPixelAt(u1, v2); // Top-left
  glm::dvec3 c22 = getPixelAt(u2, v2); // Top-right
  
  // Interpolate along bottom edge
  glm::dvec3 bottom = (1.0 - alpha) * c11 + alpha * c21;
  // Interpolate along top edge
  glm::dvec3 top = (1.0 - alpha) * c12 + alpha * c22;
  // Interpolate between top and bottom
  return (1.0 - beta) * bottom + beta * top;
}

glm::dvec3 TextureMap::getPixelAt(int x, int y) const {
  // YOUR CODE HERE
  //
  // In order to add texture mapping support to the
  // raytracer, you need to implement this function.

  // Clamp coordinates to valid range
  x = glm::clamp(x, 0, width - 1);
  y = glm::clamp(y, 0, height - 1);
  
  // Calculate the starting index in the data array for this pixel
  // Each pixel has 3 components (R,G,B) stored as uint8_t
  int index = 3 * (y * width + x);
  
  // Convert from 0-255 range to 0.0-1.0 range
  return glm::dvec3(
      data[index] / 255.0,
      data[index + 1] / 255.0,
      data[index + 2] / 255.0
  );
}

glm::dvec3 MaterialParameter::value(const isect &is) const {
  if (0 != _textureMap)
    return _textureMap->getMappedValue(is.getUVCoordinates());
  else
    return _value;
}

double MaterialParameter::intensityValue(const isect &is) const {
  if (0 != _textureMap) {
    glm::dvec3 value(_textureMap->getMappedValue(is.getUVCoordinates()));
    return (0.299 * value[0]) + (0.587 * value[1]) + (0.114 * value[2]);
  } else
    return (0.299 * _value[0]) + (0.587 * _value[1]) + (0.114 * _value[2]);
}
