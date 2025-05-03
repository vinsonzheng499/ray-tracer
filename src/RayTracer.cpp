#include "RayTracer.h"
#include "scene/light.h"
#include "scene/material.h"
#include "scene/ray.h"

#include "parser/JsonParser.h"
#include "parser/Parser.h"
#include "parser/Tokenizer.h"
#include <json.hpp>

#include "ui/TraceUI.h"
#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <string.h> // for memset

#include <fstream>
#include <iostream>

#include <thread>
#include <future>

#include <cmath>
#include <random>

using namespace std;
extern TraceUI *traceUI;

// Use this variable to decide if you want to print out debugging messages. Gets
// set in the "trace single ray" mode in TraceGLWindow, for example.
bool debugMode = false;

// Add a thread-local random number generator
thread_local std::mt19937 rng(std::hash<std::thread::id>{}(std::this_thread::get_id()));
thread_local std::uniform_real_distribution<double> distribution(0.0, 1.0);

// Helper function to get random number between 0 and 1
double getRandomDouble() {
  return distribution(rng);
}

// Generate a sample from a GGX distribution with given roughness
glm::dvec3 sampleGGX(const glm::dvec3& N, const glm::dvec3& V, double roughness, double& pdf) {
  double r1 = getRandomDouble();
  double r2 = getRandomDouble();

  // Compute half-vector
  double alpha = roughness * roughness;
  double phi = 2.0 * M_PI * r1;

  // Map from uniform to GGX distribution
  double cosTheta = sqrt((1.0 - r2) / (1.0 + (alpha*alpha - 1.0) * r2));
  double sinTheta = sqrt(1.0 - cosTheta * cosTheta);

  // Convert to Cartesian coordinates in tangent space
  glm::dvec3 H;
  H.x = sinTheta * cos(phi);
  H.y = sinTheta * sin(phi);
  H.z = cosTheta;

  // Create tangent space
  // Ensure 'up' is not parallel to 'N'
  glm::dvec3 up = abs(N.z) < 0.999 ? glm::dvec3(0, 0, 1) : glm::dvec3(1, 0, 0);
  glm::dvec3 tangent = glm::normalize(glm::cross(up, N));
  glm::dvec3 bitangent = glm::cross(N, tangent);

  // Convert from tangent space to world space
  glm::dvec3 worldH = tangent * H.x + bitangent * H.y + N * H.z;
  worldH = glm::normalize(worldH);

  // Calculate PDF for the half-vector
  double NdotH = glm::max(glm::dot(N, worldH), 0.0);

  // GGX distribution
  double alpha2 = alpha * alpha;
  double denom = NdotH * NdotH * (alpha2 - 1.0) + 1.0;
  if (denom < 1e-6) denom = 1e-6; // Avoid division by zero or very small numbers
  double D = alpha2 / (M_PI * denom * denom);

  // Convert half-vector PDF to light direction PDF
  double HdotV = glm::max(glm::dot(worldH, V), 1e-6); // Avoid division by zero

  // PDF = D * NdotH / (4 * HdotV)
  pdf = (D * NdotH) / (4.0 * HdotV);
  if (pdf < 1e-6) pdf = 1e-6; // Avoid zero PDF


  // Reflect view direction around half-vector to get light direction
  return glm::normalize(glm::reflect(-V, worldH));
}

// Calculate the full GGX BRDF
glm::dvec3 evaluateGGXBRDF(const glm::dvec3& N, const glm::dvec3& V, const glm::dvec3& L,
                        const glm::dvec3& ks, double roughness, double& pdf) {
  // Calculate half-vector
  glm::dvec3 H = glm::normalize(V + L);

  // Various dot products needed
  double NdotL = glm::max(glm::dot(N, L), 0.0);
  double NdotV = glm::max(glm::dot(N, V), 0.0);
  double NdotH = glm::max(glm::dot(N, H), 0.0);
  double HdotV = glm::max(glm::dot(H, V), 1e-6); // Avoid division by zero

  if (NdotL <= 0.0 || NdotV <= 0.0) {
      pdf = 0.0;
      return glm::dvec3(0.0);
  }

  // Roughness-related terms
  double alpha = roughness * roughness;
  double alpha2 = alpha * alpha;

  // D term (Normal distribution function)
  double denom_D = NdotH * NdotH * (alpha2 - 1.0) + 1.0;
  if (denom_D < 1e-6) denom_D = 1e-6; // Avoid division by zero or very small numbers
  double D = alpha2 / (M_PI * denom_D * denom_D);

  // G term (Geometric shadowing) - Smith's method with GGX
  double G1_V_denom = NdotV + sqrt(alpha2 + (1.0 - alpha2) * NdotV * NdotV);
  double G1_V = (G1_V_denom > 1e-6) ? (2.0 * NdotV / G1_V_denom) : 0.0;

  double G1_L_denom = NdotL + sqrt(alpha2 + (1.0 - alpha2) * NdotL * NdotL);
  double G1_L = (G1_L_denom > 1e-6) ? (2.0 * NdotL / G1_L_denom) : 0.0;

  double G = G1_V * G1_L;

  // F term (Fresnel) - Schlick approximation
  // Assume F0 is the specular color ks
  glm::dvec3 F = ks + (glm::dvec3(1.0) - ks) * pow(1.0 - HdotV, 5.0);

  // Put it all together
  // BRDF = D * G * F / (4 * NdotV * NdotL)
  double denom_BRDF = 4.0 * NdotV * NdotL;
  if (denom_BRDF < 1e-6) {
      pdf = 0.0;
      return glm::dvec3(0.0); // Avoid division by zero
  }
  glm::dvec3 brdf = (D * G * F) / denom_BRDF;

  // Calculate PDF for the half-vector for importance sampling
  // PDF = D * NdotH / (4 * HdotV)
  pdf = (D * NdotH) / (4.0 * HdotV);
  if (pdf < 1e-6) pdf = 1e-6; // Avoid zero PDF

  return brdf;
}

// Calculate Fresnel reflectance using Schlick's approximation
glm::dvec3 fresnelSchlick(const glm::dvec3& F0, double cosTheta) {
  return F0 + (glm::dvec3(1.0) - F0) * pow(glm::max(1.0 - cosTheta, 0.0), 5.0);
}

glm::dvec3 RayTracer::traceRay(ray &r, const glm::dvec3 &thresh, int maxDepth, double &t) {
  // Accumulator for the path radiance
  glm::dvec3 L(0.0, 0.0, 0.0);
  // Current path throughput (starts at white)
  glm::dvec3 throughput(1.0, 1.0, 1.0);
  // The ray for the current path segment
  ray currentRay = r;
  // Track if the last bounce was perfectly specular (or refractive)
  bool specularBounce = true; // Treat the camera ray as specular

  // Iterative Path Tracing Loop
  for (int depth = 0; depth <= maxDepth; depth++) {
      // --- 1. Check for Path Termination ---
      // Terminate if throughput is negligible
      if (glm::max(throughput.x, glm::max(throughput.y, throughput.z)) < thresh.x) // Using thresh.x as a representative threshold
          break;

      // Russian Roulette for path termination (start after a few bounces)
      if (depth > 3) {
          // Calculate survival probability based on throughput luminance
          double p = glm::max(throughput.x, glm::max(throughput.y, throughput.z));
          p = std::min(0.95, p); // Clamp probability to avoid near-certain termination

          if (getRandomDouble() > p) {
              break; // Terminate path
          }
          throughput /= p; // Compensate for terminated paths
      }


      // --- 2. Find Intersection ---
      isect i;
      if (!scene->intersect(currentRay, i)) {
          // Ray hit nothing - add environment contribution
          if (traceUI->cubeMap()) {
            CubeMap* cubeMap = traceUI->getCubeMap();
            if (cubeMap) {
              L += throughput * cubeMap->getColor(currentRay);
            }
          }
          break; // End the path
      }

      // --- 3. Prepare Surface Interaction Data ---
      const Material &material = i.getMaterial();
      glm::dvec3 hitPoint = currentRay.at(i);
      glm::dvec3 N = glm::normalize(i.getN());
      glm::dvec3 D_in = glm::normalize(currentRay.getDirection()); // Incoming direction
      glm::dvec3 V = -D_in; // Direction towards the viewer/previous bounce

      // Handle normal orientation for transparent materials / backfaces
      bool entering = (glm::dot(D_in, N) < 0.0);
      if (!entering && material.Trans()) { // Only flip for transmissive if hitting from inside
          N = -N;
      } else if (glm::dot(D_in, N) > 0.0 && !material.Trans()) { // Flip if hitting backface of opaque material
          N = -N;
      }


      // --- 4. Add Emission ---
      // Add emitted light from the current surface hit
      glm::dvec3 emission = material.ke(i);
      // Add emission contribution only if it's the first hit or coming from a specular bounce
      if (depth == 0 || specularBounce) {
          L += throughput * emission;
      }


// --- 5. Direct Lighting with MIS for Area Lights ---
if (material.kd(i).length() > 1e-6 || material.ks(i).length() > 1e-6) {
  const auto& lights = scene->getAllLights();
  if (!lights.empty()) {
      for (const Light* light : lights) {
          // Check if this is an area light
          const AreaLight* areaLight = dynamic_cast<const AreaLight*>(light);
          if (areaLight) {

            double pd = 0.0, ps = 0.0, pr = 0.0, pt = 0.0;
            {
                // Material components for sampling probabilities
                glm::dvec3 kd = material.kd(i);
                glm::dvec3 ks = material.ks(i);
                glm::dvec3 kr = material.kr(i);
                glm::dvec3 kt = material.kt(i);
            
                // Calculate probabilities based on reflectance magnitudes
                pd = glm::max(kd.x, glm::max(kd.y, kd.z)); // Diffuse probability
                ps = glm::max(ks.x, glm::max(ks.y, ks.z)); // Specular probability
                pr = glm::max(kr.x, glm::max(kr.y, kr.z)); // Perfect reflection probability
                pt = glm::max(kt.x, glm::max(kt.y, kt.z)); // Refraction probability
            
                double totalProb = pd + ps + pr + pt;
                if (totalProb > 1e-6) {
                    // Normalize probabilities
                    pd /= totalProb;
                    ps /= totalProb;
                    pr /= totalProb;
                    pt /= totalProb;
                } else {
                    // Default to diffuse if all components are near zero
                    pd = 1.0;
                    ps = pr = pt = 0.0;
                }
            }
              // === Area Light Sampling with MIS ===
              int numLightSamples = 4; // Adjust based on quality needs
              glm::dvec3 lightContribution(0.0);
              
              for (int s = 0; s < numLightSamples; ++s) {
                  // Sample a point on the light
                  glm::dvec3 lightPoint = areaLight->sample();
                  glm::dvec3 dirToLight = glm::normalize(lightPoint - hitPoint);
                  double distToLight = glm::distance(hitPoint, lightPoint);
                  
                  // Light PDF (uniform over area)
                  double lightPdf = areaLight->getPDF();
                  
                  // Cosine term at the light
                  double lightNdotL = -glm::dot(areaLight->getNormal(), dirToLight);
                  if (lightNdotL <= 0.0) continue; // Light faces away
                  
                  // Convert from area to solid angle measure
                  lightPdf *= (distToLight * distToLight) / lightNdotL;
                  if (lightPdf < 1e-6) continue;
                  
                  // Check visibility
                  ray shadowRay(hitPoint + N * RAY_EPSILON, dirToLight, glm::dvec3(1.0), ray::SHADOW);
                  isect shadowIsect;
                  bool occluded = false;
                  
                  if (scene->intersect(shadowRay, shadowIsect)) {
                      if (shadowIsect.getT() < distToLight - RAY_EPSILON) {
                          occluded = true;
                          // Handle transparency if needed
                          if (shadowIsect.getMaterial().Trans()) {
                              // Use your existing shadow attenuation code
                              glm::dvec3 shadowAtten = areaLight->shadowAttenuation(shadowRay, hitPoint);
                              if (glm::length(shadowAtten) < 0.01) // Almost completely blocked
                                  continue;
                              occluded = false;
                          } else {
                              continue; // Skip if occluded by opaque object
                          }
                      }
                  }
                  
                  if (!occluded) {
                      // NdotL at the surface
                      double NdotL = glm::max(0.0, glm::dot(N, dirToLight));
                      if (NdotL <= 0.0) continue;
                      
                      // BRDF and its PDF
                      glm::dvec3 brdf(0.0);
                      double brdfPdf = 0.0;
                      
                      // Diffuse component
                      if (material.kd(i).length() > 1e-6) {
                          brdf += material.kd(i) / M_PI;
                          brdfPdf += NdotL / M_PI * pd;
                      }
                      
                      // Specular component
                      if (material.ks(i).length() > 1e-6) {
                          double roughness = sqrt(2.0 / (2.0 + material.shininess(i)));
                          roughness = glm::clamp(roughness, 0.01, 0.99);
                          double spec_pdf;
                          glm::dvec3 brdf_spec = evaluateGGXBRDF(N, V, dirToLight, material.ks(i), roughness, spec_pdf);
                          brdf += brdf_spec;
                          brdfPdf += spec_pdf * ps;
                      }
                      
                      // MIS weight using balance heuristic
                      double weight = lightPdf / (lightPdf + brdfPdf);
                      if (!std::isfinite(weight)) weight = 0.0;
                      
                      // Add weighted contribution
                      double distAtten = light->distanceAttenuation(hitPoint);
                      lightContribution += brdf * areaLight->getColor() * NdotL * distAtten * weight;
                  }
              }
              
              // Average over samples and add to total
              if (numLightSamples > 0) {
                  L += throughput * lightContribution / static_cast<double>(numLightSamples);
              }
            } else {
              // Handle point and directional lights with NEE
              glm::dvec3 dirToLight = light->getDirection(hitPoint);
              double distToLight = std::numeric_limits<double>::infinity();
              
              // For PointLight, get the distance for attenuation 
              const PointLight* pointLight = dynamic_cast<const PointLight*>(light);
              if (pointLight) {
                  distToLight = glm::distance(pointLight->getPosition(), hitPoint);
              }
              
              // Check visibility with shadow ray
              ray shadowRay(hitPoint + N * RAY_EPSILON, dirToLight, glm::dvec3(1.0), ray::SHADOW);
              isect shadowIsect;
              bool visible = true;
              
              if (scene->intersect(shadowRay, shadowIsect)) {
                  if (shadowIsect.getT() < distToLight - RAY_EPSILON) {
                      visible = false;
                      
                      // Handle transparency in shadow rays
                      if (shadowIsect.getMaterial().Trans()) {
                          glm::dvec3 shadowAtten = light->shadowAttenuation(shadowRay, hitPoint);
                          if (glm::length(shadowAtten) >= 0.01) { // Not completely blocked
                              visible = true;
                          }
                      }
                  }
              }
              
              if (visible) {
                  double NdotL = glm::max(0.0, glm::dot(N, dirToLight));
                  if (NdotL > 0.0) {
                      // Calculate BRDF for both diffuse and specular components
                      glm::dvec3 brdf_total(0.0);
                      
                      // Diffuse component (Lambertian)
                      if (material.kd(i).length() > 1e-6) {
                          brdf_total += material.kd(i) / M_PI;
                      }
                      
                      // Specular component (GGX)
                      if (material.ks(i).length() > 1e-6) {
                          double roughness = sqrt(2.0 / (2.0 + material.shininess(i)));
                          roughness = glm::clamp(roughness, 0.01, 0.99);
                          double spec_pdf; // Not used here but needed for function
                          brdf_total += evaluateGGXBRDF(N, V, dirToLight, material.ks(i), 
                                                    roughness, spec_pdf);
                      }
                      
                      double distAtten = light->distanceAttenuation(hitPoint);
                      L += throughput * brdf_total * light->getColor() * NdotL * distAtten;
                  }
              }
          }
      }
  }
}

      // --- 6. Indirect Lighting (Importance Sampling Next Bounce) ---

      // Material properties for sampling probabilities
      glm::dvec3 kd = material.kd(i);
      glm::dvec3 ks = material.ks(i);
      glm::dvec3 kr = material.kr(i);
      glm::dvec3 kt = material.kt(i);

      // Calculate probabilities based on reflectance/transmittance magnitudes (simplified)
      double pd = glm::max(kd.x, glm::max(kd.y, kd.z)); // Diffuse probability
      double ps = glm::max(ks.x, glm::max(ks.y, ks.z)); // Specular (glossy) probability
      double pr = glm::max(kr.x, glm::max(kr.y, kr.z)); // Perfect reflection probability
      double pt = glm::max(kt.x, glm::max(kt.y, kt.z)); // Refraction probability

      double totalProb = pd + ps + pr + pt;

      if (totalProb < 1e-6) { // If material absorbs all light
          break;
      }

      // Normalize probabilities
      pd /= totalProb;
      ps /= totalProb;
      pr /= totalProb;
      pt /= totalProb;

      // Sample next event type
      double random = getRandomDouble();
      glm::dvec3 nextDir;
      double pdf = 1.0;
      glm::dvec3 brdfFactor(1.0); // Factor combining BRDF * cos(theta) / pdf
      specularBounce = false; // Reset flag for the next bounce

      if (random < pd) { // === Sample Diffuse ===
          // Create orthonormal basis
          glm::dvec3 w = N;
          glm::dvec3 u = glm::normalize(glm::cross((abs(w.x) > 0.1 ? glm::dvec3(0, 1, 0) : glm::dvec3(1, 0, 0)), w));
          glm::dvec3 v_tangent = glm::cross(w, u); // Renamed to avoid conflict

          // Cosine-weighted hemisphere sample
          double r1 = getRandomDouble();
          double r2 = getRandomDouble();
          double phi = 2.0 * M_PI * r1;
          double cosTheta_sq = r2;
          double cosTheta = sqrt(cosTheta_sq);
          double sinTheta = sqrt(1.0 - cosTheta_sq);

          // Convert to world space
          nextDir = glm::normalize(u * cos(phi) * sinTheta + v_tangent * sin(phi) * sinTheta + w * cosTheta);

          // PDF for cosine-weighted sampling = cos(theta) / pi = NdotL / pi
          double NdotL = glm::max(0.0, glm::dot(N, nextDir));
          pdf = NdotL / M_PI;
          if (pdf < 1e-6) pdf = 1e-6; // Avoid division by zero

          // BRDF = kd / pi
          // Factor = BRDF * NdotL / PDF = (kd/pi) * NdotL / (NdotL/pi) = kd
          brdfFactor = kd;

          currentRay = ray(hitPoint + N * RAY_EPSILON, nextDir, currentRay.getAtten(), ray::VISIBILITY);
          throughput *= brdfFactor / pd; // Divide by selection probability
      }
      else if (random < pd + ps && material.ks(i).length() > 1e-6) { // === Sample Glossy Specular ===
           double roughness = sqrt(2.0 / (2.0 + material.shininess(i)));
           roughness = glm::clamp(roughness, 0.01, 0.99); // Clamp roughness
           double ggx_pdf;
           nextDir = sampleGGX(N, V, roughness, ggx_pdf);
           pdf = ggx_pdf;

           if (glm::dot(nextDir, N) > 0.0 && pdf > 0.0) {
               double eval_pdf; // PDF from evaluation (should ideally match sampling PDF)
               glm::dvec3 brdf_eval = evaluateGGXBRDF(N, V, nextDir, material.ks(i), roughness, eval_pdf);
               double NdotL = glm::max(0.0, glm::dot(N, nextDir));

               brdfFactor = brdf_eval * NdotL;

               currentRay = ray(hitPoint + N * RAY_EPSILON, nextDir, currentRay.getAtten(), ray::REFLECTION);
               throughput *= brdfFactor / (pdf * ps); // Adjust throughput
               specularBounce = true; // Glossy counts as specular for env light
           } else {
               break; // Sampling failed, terminate path
           }
      }
      else if (random < pd + ps + pr && material.Refl()) { // === Sample Perfect Reflection ===
          nextDir = glm::normalize(glm::reflect(D_in, N));
          pdf = 1.0; // Delta distribution

          // Calculate Fresnel reflectance
          double cosTheta_reflect = glm::max(glm::dot(V, N), 0.0);
          glm::dvec3 F = fresnelSchlick(material.kr(i), cosTheta_reflect);

          brdfFactor = F; // For perfect reflection, BRDF * cos / PDF = Fresnel

          currentRay = ray(hitPoint + N * RAY_EPSILON, nextDir, currentRay.getAtten(), ray::REFLECTION);
          throughput *= brdfFactor / pr;
          specularBounce = true;
      }
      else if (material.Trans()) { // === Sample Refraction ===
          double n1 = 1.0;
          double n2 = material.index(i);
          if (!entering) std::swap(n1, n2);
          double eta = n1 / n2;
          double cosTheta_in = glm::dot(V, N); // V = -D_in

          double k = 1.0 - eta * eta * (1.0 - cosTheta_in * cosTheta_in);

          // Calculate Fresnel
          double cosTheta_out = sqrt(glm::max(0.0, k));
          double R0 = ((n1 - n2) / (n1 + n2)) * ((n1 - n2) / (n1 + n2));
          double fresnelProb = R0 + (1.0 - R0) * pow(1.0 - (entering ? cosTheta_in : cosTheta_out), 5.0);

          if (k < 0.0 || getRandomDouble() < fresnelProb) { // Total internal reflection or Fresnel reflection
              nextDir = glm::normalize(glm::reflect(D_in, N));
              pdf = 1.0;
              brdfFactor = material.kt(i); // Using kt for reflectance part here is complex, approximate with kt
              currentRay = ray(hitPoint + N * RAY_EPSILON, nextDir, currentRay.getAtten(), ray::REFLECTION);
              specularBounce = true;
          } else { // Refraction
              nextDir = glm::normalize(eta * D_in + (eta * cosTheta_in - cosTheta_out) * N);
              pdf = 1.0;

              // Account for radiance scaling due to change in refractive index (eta^2 term)
              // Simplified factor for throughput update: kt * scaling / probability
              brdfFactor = material.kt(i) * (eta*eta); // Scaling factor included

              currentRay = ray(hitPoint - N * RAY_EPSILON, nextDir, currentRay.getAtten(), ray::REFRACTION);
              specularBounce = true;
          }
          throughput *= brdfFactor / pt;
      }
      else { // Fallback or absorption - should ideally not happen if probabilities sum correctly
          break;
      }

      // If throughput becomes zero (or NaN/Inf), terminate
      if (!std::isfinite(throughput.x) || !std::isfinite(throughput.y) || !std::isfinite(throughput.z) || glm::length(throughput) < 1e-6) {
        break;
      }

  } // End of path tracing loop

  // Return the accumulated radiance. Clamping happens at the pixel level after averaging.
  return L;
}

// Trace a top-level ray through pixel(i,j), i.e. normalized window coordinates
// (x,y), through the projection plane, and out into the scene.
// Modified to calculate multiple samples per call based on traceUI->getSuperSamples()
glm::dvec3 RayTracer::tracePixel(int i, int j) {
  glm::dvec3 accumulated_color(0.0, 0.0, 0.0); // Accumulate color over multiple samples

  if (!sceneLoaded())
      return accumulated_color;

  // Get the target number of samples for this pixel from the UI setting
  // This was read into the 'samples' member variable during traceSetup
  int targetSamples = samples;
  if (targetSamples <= 0) targetSamples = 1; // Ensure at least one sample


  // Get pixel index and pointer once
  int pixelIndex = j * buffer_width + i;
  unsigned char *pixel = buffer.data() + pixelIndex * 3;

  // --- Loop to calculate multiple samples per pixel call ---
  for (int s = 0; s < targetSamples; ++s) {
      // Base coordinates for the pixel center
      double x_base = double(i) / double(buffer_width);
      double y_base = double(j) / double(buffer_height);

      // Get a stratified sample for jitter within the pixel
      glm::dvec2 sampleOffset = getNextSample2D(); // Returns [0,1]x[0,1] offset within subpixel

      // Calculate final sample coordinates by scaling the offset to pixel size and centering
      double x = x_base + (sampleOffset.x - 0.5) / double(buffer_width);
      double y = y_base + (sampleOffset.y - 0.5) / double(buffer_height);

      // Create and trace the ray for this sample
      ray r(glm::dvec3(0, 0, 0), glm::dvec3(0, 0, 0), glm::dvec3(1, 1, 1), ray::VISIBILITY);
      scene->getCamera().rayThrough(x, y, r);

      double dummy_t; // traceRay needs a 't' reference
      glm::dvec3 sampleColor = traceRay(r, glm::dvec3(thresh), traceUI->getDepth(), dummy_t);

      // Accumulate the color (radiance) from this sample
      accumulated_color += sampleColor;
  }
  // --- End of sample loop ---

  // Average the accumulated color over the number of samples taken in this call
  glm::dvec3 final_color = accumulated_color / static_cast<double>(targetSamples);

  // --- Update pixel buffer (still uses progressive average logic) ---
  // This part blends the average of *this pass'* samples with previous passes.
  // If you want each 'Render' click to *replace* the previous image,
  // you'd reset samplesPerPixel in traceSetup and remove the blending here.

  int currentTotalSamples = 0;
   glm::dvec3 existingBlendedColor(0.0);
  if (pixelIndex < samplesPerPixel.size()) {
      currentTotalSamples = samplesPerPixel[pixelIndex];
      if (currentTotalSamples > 0) {
           existingBlendedColor = glm::dvec3(pixel[0] / 255.0, pixel[1] / 255.0, pixel[2] / 255.0);
      }
  } else {
      return glm::dvec3(1,0,1); // Error case
  }

  // Blend the average color of *this pass* with the *previous blended* color
  // Weighted average based on the number of samples
  glm::dvec3 newlyBlendedColor;
  int totalSamplesAfterThisPass = currentTotalSamples + targetSamples;

  if (totalSamplesAfterThisPass > 0) {
       newlyBlendedColor = (existingBlendedColor * static_cast<double>(currentTotalSamples) + final_color * static_cast<double>(targetSamples)) / static_cast<double>(totalSamplesAfterThisPass);
  } else {
       newlyBlendedColor = final_color; // Should only happen if currentTotalSamples and targetSamples are 0
  }


  // Clamp the final blended color for display
  newlyBlendedColor = glm::clamp(newlyBlendedColor, 0.0, 1.0);

  // Write the clamped display color to the buffer
  pixel[0] = static_cast<unsigned char>(newlyBlendedColor.x * 255.0);
  pixel[1] = static_cast<unsigned char>(newlyBlendedColor.y * 255.0);
  pixel[2] = static_cast<unsigned char>(newlyBlendedColor.z * 255.0);

  // Update the total sample count for this pixel
  if (pixelIndex < samplesPerPixel.size()) {
      samplesPerPixel[pixelIndex] += targetSamples;
  }

  return newlyBlendedColor; // Return the display color
}


RayTracer::RayTracer()
    : scene(nullptr), buffer(0), thresh(0.001), buffer_width(0), buffer_height(0),
      m_bBufferReady(false) {} // Initialize thresh


RayTracer::~RayTracer() {
    waitRender(); // Ensure threads are joined before destruction
}

void RayTracer::getBuffer(unsigned char *&buf, int &w, int &h) {
  buf = buffer.data();
  w = buffer_width;
  h = buffer_height;
}

double RayTracer::aspectRatio() {
  return sceneLoaded() ? scene->getCamera().getAspectRatio() : 1;
}

bool RayTracer::loadScene(const char *fn) {
  ifstream ifs(fn);
  if (!ifs) {
    string msg("Error: couldn't read scene file ");
    msg.append(fn);
    traceUI->alert(msg);
    return false;
  }

  // Check if fn ends in '.ray'
  bool isRay = false;
  const char *ext = strrchr(fn, '.');
  if (ext && !strcmp(ext, ".ray"))
    isRay = true;

  // Strip off filename, leaving only the path:
  string path(fn);
  if (path.find_last_of("\\/") == string::npos)
    path = ".";
  else
    path = path.substr(0, path.find_last_of("\\/"));

  if (isRay) {
    // .ray Parsing Path (Keep existing logic)
    Tokenizer tokenizer(ifs, false);
    Parser parser(tokenizer, path);
    try {
      scene.reset(parser.parseScene());
    } catch (SyntaxErrorException &pe) {
      traceUI->alert(pe.formattedMessage());
      return false;
    } catch (ParserException &pe) {
      string msg("Parser: fatal exception ");
      msg.append(pe.message());
      traceUI->alert(msg);
      return false;
    } catch (TextureMapException e) {
      string msg("Texture mapping exception: ");
      msg.append(e.message());
      traceUI->alert(msg);
      return false;
    }
  } else {
    // JSON Parsing Path
    try {
      JsonParser parser(path, ifs);
      scene.reset(parser.parseScene());
    } catch (ParserException &pe) {
      string msg("Parser: fatal exception ");
      msg.append(pe.message());
      traceUI->alert(msg);
      return false;
    } catch (const json::exception &je) {
      string msg("Invalid JSON encountered: "); // Added colon for clarity
      msg.append(je.what());
      traceUI->alert(msg);
      return false;
    }
     catch (TextureMapException& e) { // Catch texture exceptions during JSON parsing too
            string msg("Texture mapping exception: ");
            msg.append(e.message());
            traceUI->alert(msg);
            return false;
     }
  }

  if (!sceneLoaded())
    return false;

  // Initialize camera aspect ratio if not set in scene
    if (scene->getCamera().getAspectRatio() <= 0) {
        scene->getCamera().setAspectRatio(1.0); // Default to 1.0 if not set
    }


  return true;
}

void RayTracer::traceSetup(int w, int h) {
  size_t newBufferSize = static_cast<size_t>(w) * h * 3; // Use size_t for buffer size calculation
  // Check if size changed OR if it's the first setup for this RayTracer instance
  bool sizeChanged = (newBufferSize != buffer.size() || w != buffer_width || h != buffer_height);

  if (sizeChanged || !m_bBufferReady) { // Also re-init if not ready
      bufferSize = newBufferSize;
      buffer.resize(bufferSize);
      samplesPerPixel.assign(static_cast<size_t>(w) * h, 0); // Reset sample counts when size changes
      buffer_width = w;
      buffer_height = h;
      // *** Add this line back to clear the buffer ***
      std::fill(buffer.begin(), buffer.end(), 0);
      m_bBufferReady = true; // Mark as ready
  } else {
      // If size hasn't changed, we still need to reset sample counts for a new render pass
      // unless we want full progressive rendering across multiple "Render" clicks.
      // For now, let's reset samples to ensure each "Render" click starts fresh sampling counts,
      // matching the expectation of clearing the canvas.
       samplesPerPixel.assign(static_cast<size_t>(w) * h, 0);
       // *** Also clear the buffer here to ensure canvas clears even if size is the same ***
       std::fill(buffer.begin(), buffer.end(), 0);
  }


  /*
   * Sync with TraceUI
   */
  threads = traceUI->getThreads();
  block_size = traceUI->getBlockSize();
  thresh = traceUI->getThreshold();
  samples = traceUI->getSuperSamples(); // Read samples per pixel for the upcoming pass
  aaThresh = traceUI->getAaThreshold();

  bvhMaxDepth = traceUI->getMaxDepth();
  bvhTargetLeafSize = traceUI->getLeafSize();

  // Regenerate samples for the new pass based on the UI setting
  generateStratifiedSamples(samples);

  // BVH setup
  if (traceUI->bvhSwitch()) {
    // Avoid rebuilding BVH if the scene hasn't changed.
    // This requires tracking scene changes, which isn't implemented here.
    // For simplicity, we rebuild it if the switch is on.
    scene->buildBVH(bvhMaxDepth, bvhTargetLeafSize);
  } else {
    scene->clearBVH();
  }
}

void RayTracer::workerThread(int /*threadId*/) { // threadId might not be needed anymore
    while (true) {
        Pixel pixel(0, 0, nullptr);

        // Get next pixel from queue
        {
            std::lock_guard<std::mutex> lock(bufferMutex);
            if (pixelQueue.empty()) {
                 // No more pixels for this frame/iteration
                 // Instead of setting threadDone, just exit the loop
                 return;
            }
            pixel = pixelQueue.front();
            pixelQueue.pop();
        }

        // Process the pixel (calculate ONE sample and blend)
        // tracePixel now handles the blending internally
        tracePixel(pixel.ix, pixel.jy);

        // Atomically increment the counter for completed pixels for this frame/iteration
        completedPixels++;

        // Optional: Check stopTrace flag periodically if needed
        if (stopTrace) {
            return; // Exit if stop requested
        }
    }
}


void RayTracer::traceImage(int w, int h) {
    // Setup only needs to happen if size changed or first time
    traceSetup(w, h);

    // Clear any existing threads first to prevent issues
    waitRender(); // Ensure previous render is complete
    workerThreads.clear();
    threadDone.assign(threads, false); // Reset thread done status


    // Ensure the queue is empty before repopulating
    std::lock_guard<std::mutex> lock(bufferMutex); // Lock mutex before clearing queue
    while (!pixelQueue.empty()) {
        pixelQueue.pop();
    }

    // Create a list of all pixel coordinates
    std::vector<std::pair<int, int>> pixelCoords;
    pixelCoords.reserve(static_cast<size_t>(w) * h);
    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            pixelCoords.push_back({i, j});
        }
    }

    // Shuffle pixel order for better visual feedback during progressive rendering
    std::shuffle(pixelCoords.begin(), pixelCoords.end(), rng);

    // Populate the pixel queue with shuffled coordinates
    for (const auto& coord : pixelCoords) {
        unsigned char* pixelPtr = buffer.data() + (coord.first + coord.second * w) * 3;
        pixelQueue.push(Pixel(coord.first, coord.second, pixelPtr));
    }

    completedPixels = 0; // Reset completed pixel count for this frame/iteration
    renderingDone = false; // Mark rendering as not done
    stopTrace = false;     // Reset stop flag

    // Start the worker threads
    for (unsigned int i = 0; i < threads; ++i) {
        workerThreads.emplace_back(&RayTracer::workerThread, this, i); // Use emplace_back
    }

    // Rendering starts asynchronously. The UI loop will call checkRender().
}


int RayTracer::aaImage() {
  // Anti-aliasing is now handled implicitly by multi-sampling in tracePixel/traceImage.
  // This function might be repurposed or removed for a pure path tracer.
  // If you need a separate AA pass (e.g., for filtering), implement it here.
  // For now, it does nothing.
  return 0;
}

bool RayTracer::checkRender() {
     // Check if all pixels for the current frame/iteration have been processed
     if (completedPixels >= buffer_width * buffer_height) {
         // Optional: Automatically join threads here if you want the check to block until frame completion
         // waitRender();
         renderingDone = true;
         return true;
     }
     return renderingDone; // Return true only if explicitly marked done or all pixels processed
}


void RayTracer::waitRender() {
  for (auto& thread : workerThreads) {
      if (thread.joinable()) {
          thread.join();
      }
  }
   // Clear threads only after joining all of them
   workerThreads.clear();
   threadDone.clear(); // Clear status tracking
   renderingDone = true; // Mark as done after joining
}


glm::dvec3 RayTracer::getPixel(int i, int j) {
    if (i < 0 || i >= buffer_width || j < 0 || j >= buffer_height) {
       return glm::dvec3(0.0); // Or handle error appropriately
    }
    unsigned char* pixel = buffer.data() + (i + j * buffer_width) * 3;
    return glm::dvec3(pixel[0] / 255.0, pixel[1] / 255.0, pixel[2] / 255.0);
}

void RayTracer::setPixel(int i, int j, glm::dvec3 color) {
     if (i < 0 || i >= buffer_width || j < 0 || j >= buffer_height) {
        return; // Or handle error appropriately
     }
    unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;

    // Clamp color values before casting to unsigned char
    pixel[0] = static_cast<unsigned char>(glm::clamp(color.x * 255.0, 0.0, 255.0));
    pixel[1] = static_cast<unsigned char>(glm::clamp(color.y * 255.0, 0.0, 255.0));
    pixel[2] = static_cast<unsigned char>(glm::clamp(color.z * 255.0, 0.0, 255.0));
}

void RayTracer::generateStratifiedSamples(int totalSamples) { // Parameter is total samples now
    if (totalSamples <= 0) return;

    stratifiedSamples.resize(totalSamples);
    int samplesPerDimension = static_cast<int>(std::sqrt(static_cast<double>(totalSamples)));
    // Adjust samplesPerDimension if totalSamples is not a perfect square,
    // though for simplicity we often stick to square numbers.
    if (samplesPerDimension * samplesPerDimension != totalSamples) {
       // Handle non-square sample counts if necessary, e.g., use a 1D stratification.
       // For now, we'll proceed assuming it's close enough or a perfect square.
       samplesPerDimension = static_cast<int>(round(sqrt(static_cast<double>(totalSamples))));
    }
    if(samplesPerDimension == 0) samplesPerDimension = 1; // Avoid division by zero

    int current_sample = 0;
    double subpixel_width = 1.0 / samplesPerDimension;

    for (int y = 0; y < samplesPerDimension && current_sample < totalSamples; ++y) {
        for (int x = 0; x < samplesPerDimension && current_sample < totalSamples; ++x) {
            double jitterX = getRandomDouble();
            double jitterY = getRandomDouble();

            stratifiedSamples[current_sample++] = glm::dvec2(
                (x + jitterX) * subpixel_width,
                (y + jitterY) * subpixel_width
            );
        }
    }

    // Fill remaining samples if not a perfect square (simple approach)
    while(current_sample < totalSamples) {
         stratifiedSamples[current_sample++] = glm::dvec2(getRandomDouble(), getRandomDouble());
    }


    // Shuffle the entire set of samples
    std::shuffle(stratifiedSamples.begin(), stratifiedSamples.end(), rng);

    currentSampleIndex = 0;
}


glm::dvec2 RayTracer::getNextSample2D() {
    // Cycle through the pre-generated stratified samples
    if (stratifiedSamples.empty()) {
       // Should not happen if generateStratifiedSamples was called, but handle defensively
       return glm::dvec2(getRandomDouble(), getRandomDouble());
    }
    if (currentSampleIndex >= stratifiedSamples.size()) {
       currentSampleIndex = 0; // Wrap around or reshuffle if needed
        // Optionally reshuffle here: std::shuffle(stratifiedSamples.begin(), stratifiedSamples.end(), rng);
    }
    return stratifiedSamples[currentSampleIndex++];
}
