#include <cmath>
#include <iostream>

#include "light.h"
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <iomanip>
#include <random>

using namespace std;

extern bool debugMode;

thread_local std::mt19937 light_rng(std::random_device{}());
thread_local std::uniform_real_distribution<double> light_dist(0.0, 1.0);

double DirectionalLight::distanceAttenuation(const glm::dvec3 &) const {
  // distance to light is infinite, so f(di) goes to 0.  Return 1.
  return 1.0;
}

glm::dvec3 DirectionalLight::shadowAttenuation(const ray &r, const glm::dvec3 &p) const {
    glm::dvec3 attenuation(1.0, 1.0, 1.0);
    glm::dvec3 D = glm::normalize(getDirection(p));
    ray shadowRay = ray(p + D * RAY_EPSILON, D, r.getAtten(), ray::SHADOW);
    isect i;
    
    while (scene->intersect(shadowRay, i)) {  
        if (debugMode) {
          // cout << "t: " << i.getT() << endl;
          // std::cout << std::fixed << std::setprecision(8);
          // cout << "direction light" << endl;
          // cout << "p " << p << endl;
          // cout << "getDirection(p)" << getDirection(p) << endl;
          // cout << "p + getDirection(p) * RAY_EPSILON " << p + getDirection(p) * RAY_EPSILON << endl;
          // cout << "shadowRay intersection " << shadowRay.at(i.getT()) << endl;
        }
        
        const Material &m = i.getMaterial();
        
        if (!m.Trans()) {
            // if (debugMode) {
            //   cout << "breaking early" << endl;
            // }
            return glm::dvec3(0.0, 0.0, 0.0);
        }

        D = glm::normalize(getDirection(shadowRay.getPosition()));
        glm::dvec3 N = glm::normalize(i.getN());
        bool entering = (glm::dot(D, N) < 0.0);
        if (entering) {
          N = -N;
        }

        // if (debugMode) {
        //   cout << "N: " << N << endl;
        //   cout << "(entering ? -N : N) " << (entering ? -N : N) << endl;
        //   cout << "D: " << D << endl;
        // }

        double d = glm::distance(shadowRay.getPosition(), shadowRay.at(i.getT()));
        // if (debugMode){
        //   cout << "shadow d: " << d << endl; 
        //   cout << endl;
        // }
        if (!entering) {
          attenuation *= glm::pow(m.kt(i), glm::dvec3(d));
        }
        glm::dvec3 newOrigin = shadowRay.at(i.getT()) + RAY_EPSILON * D;
        shadowRay = ray(newOrigin, D, attenuation, ray::SHADOW);
    }

    // if (debugMode) {
    //   cout << "attenuation: " << attenuation << endl;
    //   cout << endl;
    // }
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

glm::dvec3 PointLight::shadowAttenuation(const ray &r, const glm::dvec3 &p) const {
    glm::dvec3 attenuation(1.0, 1.0, 1.0);
    glm::dvec3 D = glm::normalize(getDirection(p));
    ray shadowRay = ray(p + D * RAY_EPSILON, D, r.getAtten(), ray::SHADOW);
    isect i;
    
    while (scene->intersect(shadowRay, i)) {  
        // Check if intersection is beyond the light source
        if (i.getT() > glm::distance(p, position)) {
            break;
        }

        const Material &m = i.getMaterial();
        
        if (!m.Trans()) {
            // if (debugMode) {
            //   cout << "breaking early" << endl;
            // }
            return glm::dvec3(0.0, 0.0, 0.0);
        }

        D = glm::normalize(getDirection(shadowRay.getPosition()));
        glm::dvec3 N = glm::normalize(i.getN());
        bool entering = (glm::dot(D, N) < 0.0);
        if (entering) {
          N = -N;
        }

        // if (debugMode) {
        //   cout << "shadow ray origin: " << shadowRay.getPosition() << endl;
        //   cout << "shadow ray direction" << shadowRay.getDirection() << endl;
        //   cout << "shadow ray intersection " << shadowRay.at(i.getT()) << endl;
        // }
        

        // if (debugMode) {
        //   cout << "N: " << N << endl;
        //   cout << "(entering ? -N : N) " << (entering ? -N : N) << endl;
        //   cout << "D: " << D << endl;
        // }

        double d = glm::distance(shadowRay.getPosition(), shadowRay.at(i.getT()));
        // if (debugMode){
        //   cout << "shadow d: " << d << endl; 
        //   cout << endl;
        // }
        if (!entering) {
          attenuation *= glm::pow(m.kt(i), glm::dvec3(d));
        }
        glm::dvec3 newOrigin = shadowRay.at(i.getT()) + RAY_EPSILON * D;
        shadowRay = ray(newOrigin, D, attenuation, ray::SHADOW);
    }

    // if (debugMode) {
    //   cout << "attenuation: " << attenuation << endl;
    //   cout << endl;
    // }
    return attenuation;
}

// Add these implementations after PointLight methods

double getRandomLightDouble() {
  return light_dist(light_rng);
}

glm::dvec3 AreaLight::sample() const {
  // Generate random coordinates in [0,1] x [0,1]
  double u = getRandomLightDouble();
  double v = getRandomLightDouble();
  
  // Map to area light surface
  return position + (u - 0.5) * u_len * u_dir + (v - 0.5) * v_len * v_dir;
}

glm::dvec3 AreaLight::getDirection(const glm::dvec3 &P) const {
  // For shadow rays, we'll sample a point and return direction to that point
  // For direct lighting in MIS, the caller should use sample() and handle direction
  glm::dvec3 lightPoint = sample();
  return glm::normalize(lightPoint - P);
}

glm::dvec3 AreaLight::getColor() const {
  return color;
}

double AreaLight::distanceAttenuation(const glm::dvec3 &P) const {
  // We'll use the sampled point's distance for attenuation
  // This is called after getDirection, which samples a point on the light
  glm::dvec3 lightPoint = sample(); // Ideally we'd cache this, but for simplicity we're resampling
  double distance = glm::distance(lightPoint, P);
  double attenuation = 1.0 / (constantTerm + linearTerm * distance + quadraticTerm * distance * distance);
  return glm::clamp(attenuation, 0.0, 1.0);
}

glm::dvec3 AreaLight::shadowAttenuation(const ray &r, const glm::dvec3 &p) const {
  glm::dvec3 attenuation(1.0, 1.0, 1.0);
  
  // Get a specific point on the light to test visibility to
  glm::dvec3 lightPoint = sample();
  glm::dvec3 dirToLight = glm::normalize(lightPoint - p);
  double distToLight = glm::distance(p, lightPoint);
  
  ray shadowRay = ray(p + dirToLight * RAY_EPSILON, dirToLight, r.getAtten(), ray::SHADOW);
  isect i;
  
  while (scene->intersect(shadowRay, i)) {
      // Check if intersection is beyond the light source
      if (i.getT() > distToLight - RAY_EPSILON) {
          break;
      }

      const Material &m = i.getMaterial();
      if (!m.Trans()) {
          return glm::dvec3(0.0, 0.0, 0.0);
      }

      // Handle transparent materials as before
      glm::dvec3 D = glm::normalize(dirToLight);
      glm::dvec3 N = glm::normalize(i.getN());
      bool entering = (glm::dot(D, N) < 0.0);
      if (entering) N = -N;

      double d = glm::distance(shadowRay.getPosition(), shadowRay.at(i.getT()));
      if (!entering) {
          attenuation *= glm::pow(m.kt(i), glm::dvec3(d));
      }
      glm::dvec3 newOrigin = shadowRay.at(i.getT()) + RAY_EPSILON * D;
      shadowRay = ray(newOrigin, D, attenuation, ray::SHADOW);
  }

  return attenuation;
}

#define VERBOSE 0

