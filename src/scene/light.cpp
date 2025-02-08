#include <cmath>
#include <iostream>

#include "light.h"
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>

using namespace std;

double DirectionalLight::distanceAttenuation(const glm::dvec3 &) const {
  // distance to light is infinite, so f(di) goes to 0.  Return 1.
  return 1.0;
}

glm::dvec3 DirectionalLight::shadowAttenuation(const ray &r,
                                               const glm::dvec3 &p) const {
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.
  glm::dvec3 attenuation(1.0, 1.0, 1.0);
  ray shadowRay(p + getDirection(p) * RAY_EPSILON, getDirection(p), r.getAtten(), ray::SHADOW);
  isect i;
  while (scene->intersect(shadowRay, i)) {  
    // If there's an intersection, check material transparency
    const Material &m = i.getMaterial();
    glm::dvec3 kt = m.kt(i);  // Transmittance of material at intersection

    if (glm::length(kt) == 0.0) {  
      // If kt is zero, the object is fully opaque -> fully shadowed
      return glm::dvec3(0.0, 0.0, 0.0);
    }

   // Entry point A
    glm::dvec3 A = shadowRay.at(i.getT());

    // Fire a new ray (r2) from slightly past A to find exit point B
    ray exitRay(A + getDirection(p) * RAY_EPSILON, getDirection(p), r.getAtten(), ray::SHADOW);
    isect exitIntersection;
    double d = 0.0;

    if (scene->intersect(exitRay, exitIntersection)) {
      glm::dvec3 B = exitRay.at(exitIntersection.getT());
      d = glm::distance(A, B);
    }

    // Apply component-wise attenuation: (kt)^d
    attenuation *= glm::pow(kt, glm::dvec3(d));

    // Move the shadow ray forward slightly past the intersection
    glm::dvec3 newOrigin = shadowRay.at(i.getT()) + RAY_EPSILON * getDirection(p);
    shadowRay = ray(newOrigin, getDirection(p), attenuation, ray::SHADOW);
  }

  return attenuation;
}

glm::dvec3 DirectionalLight::getColor() const { return color; }

glm::dvec3 DirectionalLight::getDirection(const glm::dvec3 &) const {
  return -orientation;
}

double PointLight::distanceAttenuation(const glm::dvec3 &P) const {
  // YOUR CODE HERE

  // You'll need to modify this method to attenuate the intensity
  // of the light based on the distance between the source and the
  // point P.  For now, we assume no attenuation and just return 1.0
  double distance = glm::distance(position, P);
  double attenuation = 1 / (constantTerm + linearTerm * distance + quadraticTerm * distance * distance);
  return glm::clamp(attenuation, 0.0, 1.0);
}

glm::dvec3 PointLight::getColor() const { return color; }

glm::dvec3 PointLight::getDirection(const glm::dvec3 &P) const {
  return glm::normalize(position - P);
}

glm::dvec3 PointLight::shadowAttenuation(const ray &r,
                                         const glm::dvec3 &p) const {
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.
  glm::dvec3 attenuation(1.0, 1.0, 1.0);
  ray shadowRay = ray(p + getDirection(p) * RAY_EPSILON, getDirection(p), r.getAtten(), ray::SHADOW);
  isect i;
  while (scene->intersect(shadowRay, i)) {  
    if (i.getT() > glm::distance(p, position)) {
      break;
    }
    // If there's an intersection, check material transparency
    const Material &m = i.getMaterial();
    glm::dvec3 kt = m.kt(i);  // Transmittance of material at intersection

    if (glm::length(kt) == 0.0) {  
      // If kt is zero, the object is fully opaque -> fully shadowed
      return glm::dvec3(0.0, 0.0, 0.0);
    }

    // Entry point A
    glm::dvec3 A = shadowRay.at(i.getT());

    // Fire a new ray (r2) from slightly past A to find exit point B
    ray exitRay(A + getDirection(p) * RAY_EPSILON, getDirection(p), r.getAtten(), ray::SHADOW);
    isect exitIntersection;
    double d = 0.0;

    if (scene->intersect(exitRay, exitIntersection)) {
      glm::dvec3 B = exitRay.at(exitIntersection.getT());
      d = glm::distance(A, B);
    }

    // Apply component-wise attenuation: (kt)^d
    attenuation *= glm::pow(kt, glm::dvec3(d));

    // Move the shadow ray forward slightly past the intersection
    glm::dvec3 newOrigin = shadowRay.at(i.getT()) + RAY_EPSILON * getDirection(p);
    shadowRay = ray(newOrigin, getDirection(p), attenuation, ray::SHADOW);
  }

  return attenuation;
}

#define VERBOSE 0

