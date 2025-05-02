#ifndef __LIGHT_H__
#define __LIGHT_H__

#ifndef _WIN32
#include <algorithm>
using std::max;
using std::min;
#endif

#include "../ui/TraceUI.h"
#include "scene.h"
#include <FL/gl.h>

class Light : public SceneElement {
public:
  virtual glm::dvec3 shadowAttenuation(const ray &r,
                                       const glm::dvec3 &pos) const = 0;
  virtual double distanceAttenuation(const glm::dvec3 &P) const = 0;
  virtual glm::dvec3 getColor() const = 0;
  virtual glm::dvec3 getDirection(const glm::dvec3 &P) const = 0;


protected:
  Light(Scene *scene, const glm::dvec3 &col)
      : SceneElement(scene), color(col) {}

  glm::dvec3 color;

public:
  virtual void glDrawLight([[maybe_unused]] GLenum lightID) const {}
  virtual void glDrawLight() const {}
};

class DirectionalLight : public Light {
public:
  DirectionalLight(Scene *scene, const glm::dvec3 &orien,
                   const glm::dvec3 &color)
      : Light(scene, color), orientation(glm::normalize(orien)) {}
  virtual glm::dvec3 shadowAttenuation(const ray &r,
                                       const glm::dvec3 &pos) const;
  virtual double distanceAttenuation(const glm::dvec3 &P) const;
  virtual glm::dvec3 getColor() const;
  virtual glm::dvec3 getDirection(const glm::dvec3 &P) const;

protected:
  glm::dvec3 orientation;

public:
  void glDrawLight(GLenum lightID) const;
  void glDrawLight() const;
};

class PointLight : public Light {
public:
  PointLight(Scene *scene, const glm::dvec3 &pos, const glm::dvec3 &color,
             float constantAttenuationTerm, float linearAttenuationTerm,
             float quadraticAttenuationTerm)
      : Light(scene, color), position(pos),
        constantTerm(constantAttenuationTerm),
        linearTerm(linearAttenuationTerm),
        quadraticTerm(quadraticAttenuationTerm) {}

  virtual glm::dvec3 shadowAttenuation(const ray &r,
                                       const glm::dvec3 &pos) const;
  virtual double distanceAttenuation(const glm::dvec3 &P) const;
  virtual glm::dvec3 getColor() const;
  virtual glm::dvec3 getDirection(const glm::dvec3 &P) const;

  void setAttenuationConstants(float a, float b, float c) {
    constantTerm = a;
    linearTerm = b;
    quadraticTerm = c;
  }

protected:
  glm::dvec3 position;

  // These three values are the a, b, and c in the distance attenuation function
  // (from the slide labelled "Intensity drop-off with distance"):
  //    f(d) = min( 1, 1/( a + b d + c d^2 ) )
  float constantTerm;  // a
  float linearTerm;    // b
  float quadraticTerm; // c

public:
  void glDrawLight(GLenum lightID) const;
  void glDrawLight() const;
  glm::dvec3 getPosition() const { return position; }

protected:
};

// After the PointLight class definition, add:
class AreaLight : public Light {
  public:
    AreaLight(Scene *scene, const glm::dvec3 &pos, const glm::dvec3 &u_axis, 
             const glm::dvec3 &v_axis, const glm::dvec3 &color,
             float constantAttenuationTerm = 0.0f, 
             float linearAttenuationTerm = 0.0f,
             float quadraticAttenuationTerm = 1.0f)
        : Light(scene, color), position(pos), u_dir(glm::normalize(u_axis)), 
          v_dir(glm::normalize(v_axis)), u_len(glm::length(u_axis)), 
          v_len(glm::length(v_axis)), 
          constantTerm(constantAttenuationTerm),
          linearTerm(linearAttenuationTerm),
          quadraticTerm(quadraticAttenuationTerm) {
            normal = glm::normalize(glm::cross(u_dir, v_dir));
            area = u_len * v_len;
          }
  
    virtual glm::dvec3 shadowAttenuation(const ray &r, const glm::dvec3 &pos) const;
    virtual double distanceAttenuation(const glm::dvec3 &P) const;
    virtual glm::dvec3 getColor() const;
    virtual glm::dvec3 getDirection(const glm::dvec3 &P) const;
  
    // Sample a random point on the area light
    glm::dvec3 sample() const;
    
    // Get PDF value for a point on the light
    double getPDF() const { return 1.0 / area; }
    
    // Get the normal of the light surface
    glm::dvec3 getNormal() const { return normal; }
  
  protected:
    glm::dvec3 position;  // Center position
    glm::dvec3 u_dir;     // Normalized u-direction
    glm::dvec3 v_dir;     // Normalized v-direction  
    glm::dvec3 normal;    // Surface normal
    double u_len;         // Length of u-axis
    double v_len;         // Length of v-axis
    double area;          // Area of light
  
    // Attenuation constants
    float constantTerm;
    float linearTerm;
    float quadraticTerm;
  
  public:
    void glDrawLight(GLenum lightID) const;
    void glDrawLight() const;
  };

#endif // __LIGHT_H__
