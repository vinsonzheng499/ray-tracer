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
    
    // If material is opaque, full shadow
    if (!m.Trans()) {
      return glm::dvec3(0.0, 0.0, 0.0);
    }

    // Check if we're entering or inside the object
    glm::dvec3 D = glm::normalize(shadowRay.getDirection());
    glm::dvec3 N = glm::normalize(i.getN());
    bool entering = (glm::dot(D, N) < 0.0);

    if (entering) {
      // Find exit point when entering object
      glm::dvec3 A = shadowRay.at(i.getT());
      ray exitRay(A - N * RAY_EPSILON, D, r.getAtten(), ray::SHADOW);
      isect exitIntersection;
      
      if (scene->intersect(exitRay, exitIntersection)) {
        glm::dvec3 B = exitRay.at(exitIntersection.getT());
        double d = glm::distance(A, B);
        // Only apply attenuation when traveling through object
        attenuation *= glm::pow(m.kt(i), glm::dvec3(d));
      }
    }

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
    
    // If material is opaque, full shadow
    if (!m.Trans()) {
      return glm::dvec3(0.0, 0.0, 0.0);
    }

    // Check if we're entering or inside the object
    glm::dvec3 D = glm::normalize(shadowRay.getDirection());
    glm::dvec3 N = glm::normalize(i.getN());
    bool entering = (glm::dot(D, N) < 0.0);

    if (entering) {
      // Find exit point when entering object
      glm::dvec3 A = shadowRay.at(i.getT());
      ray exitRay(A - N * RAY_EPSILON, D, r.getAtten(), ray::SHADOW);
      isect exitIntersection;
      
      if (scene->intersect(exitRay, exitIntersection)) {
        glm::dvec3 B = exitRay.at(exitIntersection.getT());
        double d = glm::distance(A, B);
        // Only apply attenuation when traveling through object
        attenuation *= glm::pow(m.kt(i), glm::dvec3(d));
      }
    }

    // Move the shadow ray forward slightly past the intersection
    glm::dvec3 newOrigin = shadowRay.at(i.getT()) + RAY_EPSILON * getDirection(p);
    shadowRay = ray(newOrigin, getDirection(p), attenuation, ray::SHADOW);
  }

  return attenuation;
}

#define VERBOSE 0

