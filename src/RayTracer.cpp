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

// Trace a top-level ray through pixel(i,j), i.e. normalized window coordinates
// (x,y), through the projection plane, and out into the scene. All we do is
// enter the main ray-tracing method, getting things started by plugging in an
// initial ray weight of (0.0,0.0,0.0) and an initial recursion depth of 0.

glm::dvec3 RayTracer::trace(double x, double y) {
  // Clear out the ray cache in the scene for debugging purposes
  if (TraceUI::m_debug) {
    scene->clearIntersectCache();
  }
  
  // Get the actual sample count from UI
  int samplesPerPixel = traceUI->getSuperSamples();
  if (samplesPerPixel <= 0) samplesPerPixel = 1; // Ensure at least one sample
  
  glm::dvec3 pixelColor(0.0, 0.0, 0.0);
  
  for (int s = 0; s < samplesPerPixel; s++) {
    // Get stratified sample for jitter
    glm::dvec2 sample = getNextSample2D();
    
    // Apply jitter within the pixel
    double jitterX = (sample.x - 0.5) / buffer_width;
    double jitterY = (sample.y - 0.5) / buffer_height;
    
    ray r(glm::dvec3(0, 0, 0), glm::dvec3(0, 0, 0), glm::dvec3(1, 1, 1),
          ray::VISIBILITY);
    scene->getCamera().rayThrough(x + jitterX, y + jitterY, r);
    
    double dummy;
    pixelColor += traceRay(r, glm::dvec3(1.0, 1.0, 1.0), traceUI->getDepth(), dummy);
  }
  
  // Average the samples
  pixelColor /= static_cast<double>(samplesPerPixel);
  
  return glm::clamp(pixelColor, 0.0, 1.0);
}
// Generate a sample from a GGX distribution with given roughness
glm::dvec3 sampleGGX(const glm::dvec3& N, const glm::dvec3& V, double roughness) {
  double r1 = static_cast<double>(rand()) / RAND_MAX;
  double r2 = static_cast<double>(rand()) / RAND_MAX;
  
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
  glm::dvec3 up = abs(N.z) < 0.999 ? glm::dvec3(0, 0, 1) : glm::dvec3(1, 0, 0);
  glm::dvec3 tangent = glm::normalize(glm::cross(up, N));
  glm::dvec3 bitangent = glm::cross(N, tangent);
  
  // Convert from tangent space to world space
  glm::dvec3 worldH = tangent * H.x + bitangent * H.y + N * H.z;
  
  // Reflect view direction around half-vector to get light direction
  return glm::reflect(-V, worldH);
}

// Compute GGX PDF for a given direction
double ggxPDF(const glm::dvec3& N, const glm::dvec3& H, const glm::dvec3& V, double roughness) {
  double alpha = roughness * roughness;
  double NdotH = glm::max(glm::dot(N, H), 0.0);
  double HdotV = glm::max(glm::dot(H, V), 0.0);
  
  // GGX distribution
  double alpha2 = alpha * alpha;
  double denom = NdotH * NdotH * (alpha2 - 1.0) + 1.0;
  double D = alpha2 / (M_PI * denom * denom);
  
  // PDF = D * NdotH / (4 * HdotV)
  return D * NdotH / (4.0 * HdotV);
}

glm::dvec3 RayTracer::traceRay(ray &r, const glm::dvec3 &thresh, int maxDepth, double &t) {
  // Path tracing accumulator
  glm::dvec3 L(0.0, 0.0, 0.0);    // Accumulated radiance
  glm::dvec3 throughput(1.0, 1.0, 1.0); // Path throughput
  ray currentRay = r;              // Current ray being traced
  bool specularBounce = false;     // Track if last bounce was specular

  // Iterative path tracing loop (replacing recursive calls)
  for (int depth = 0; depth <= maxDepth; depth++) {
    // Early termination if throughput is too low
    if (glm::all(glm::lessThan(throughput, thresh)))
      break;

    // Trace the current ray segment
    isect i;
    if (!scene->intersect(currentRay, i)) {
      // Ray hit nothing - environment contribution
      if (depth == 0 || specularBounce) {
        // Add environment contribution for camera rays or perfectly specular bounces
        if (traceUI->cubeMap()) {
          CubeMap* cubeMap = traceUI->getCubeMap();
          if (cubeMap) {
            L += throughput * cubeMap->getColor(currentRay);
          }
        } else {
          // Add default environment lighting for rays that miss geometry
          // This is critical - missing rays should contribute some light (sky/ambient)
          L += throughput * glm::dvec3(0.15, 0.15, 0.25); // Add a subtle blue sky color
        }
      }
      break; // End the path
    }

    // Update intersection distance for the calling function
    t = i.getT();

    // Get the material at the intersection
    const Material &material = i.getMaterial();
    glm::dvec3 hitPoint = currentRay.at(i);
    glm::dvec3 N = glm::normalize(i.getN());
    glm::dvec3 D = glm::normalize(currentRay.getDirection());

    // Handle normal orientation based on ray direction
    bool entering = (glm::dot(D, N) < 0.0);
    if (!entering) N = -N;

    // Add emitted light for all bounces (crucial for path tracing)
    glm::dvec3 emission = material.ke(i);
    if (glm::dot(emission, emission) > 0.0) {
      L += throughput * emission;
    }

    // Calculate material component weights for importance sampling
    double krLength = glm::length(material.kr(i));
    double ktLength = glm::length(material.kt(i));
    double ksLength = glm::length(material.ks(i));
    double kdLength = glm::length(material.kd(i));

    // Compute probabilities for each component
    double totalWeight = krLength + ktLength + ksLength + kdLength;
    if (totalWeight <= 0.0) {
      break; // No reflection properties, terminate path
    }
    
    // Calculate sampling probabilities
    double probReflect = krLength / totalWeight;
    double probRefract = ktLength / totalWeight;
    double probSpecular = ksLength / totalWeight;
    double probDiffuse = kdLength / totalWeight;
    
    // Normalize to sum to 1
    double sum = probReflect + probRefract + probSpecular + probDiffuse;
    if (sum > 0.0) {
      probReflect /= sum;
      probRefract /= sum;
      probSpecular /= sum;
      probDiffuse /= sum;
    } else {
      // Default to diffuse if nothing else
      probDiffuse = 1.0;
      probReflect = probRefract = probSpecular = 0.0;
    }

    // Add direct lighting for non-specular surfaces (MIS)
    if (kdLength > 0.0) {
      // Sample lights (light sampling strategy)
      const auto& lights = scene->getAllLights();
      if (!lights.empty()) {
        // Sample all lights for better convergence
        double lightContrib = 2.0; // Adjust this multiplier if needed
        for (const Light* light : lights) {
          glm::dvec3 dirToLight = light->getDirection(hitPoint);
          glm::dvec3 lightColor = light->getColor() * lightContrib;
          double distAtten = light->distanceAttenuation(hitPoint);
          
          // Check visibility
          ray shadowRay(hitPoint + N * RAY_EPSILON, dirToLight, glm::dvec3(1,1,1), ray::SHADOW);
          glm::dvec3 shadowAttenuation = light->shadowAttenuation(shadowRay, hitPoint);
          
          if (glm::length(shadowAttenuation) > 0.0) {
            // Calculate geometry term
            double cosToLight = glm::max(0.0, glm::dot(N, dirToLight));
            
            // Calculate BRDF
            glm::dvec3 brdf = material.kd(i) / M_PI;
            
            // Add direct lighting contribution
            L += throughput * brdf * lightColor * cosToLight * distAtten * shadowAttenuation;
          }
        }
      }
    }
    
    // Randomly select which BRDF to sample based on material properties
    double random = static_cast<double>(rand()) / RAND_MAX;
    double accumulatedProb = 0.0;
    
    specularBounce = false;  // Reset specular flag

    if (random < (accumulatedProb += probReflect) && material.Refl()) {
      // Perfect specular reflection
      glm::dvec3 R = glm::normalize(glm::reflect(D, N));
      currentRay = ray(hitPoint + N * RAY_EPSILON, R, currentRay.getAtten(), ray::REFLECTION);
      throughput *= material.kr(i) / probReflect;
      specularBounce = true;
    }
    else if (random < (accumulatedProb += probRefract) && material.Trans()) {
      // Refraction with Fresnel effects
      double n1 = 1.0;  // Air index of refraction
      double n2 = material.index(i);

      if (!entering) {
        std::swap(n1, n2);
      }

      double eta = n1 / n2;
      glm::dvec3 T = glm::normalize(glm::refract(D, N, eta));

      if (glm::length(T) > 0.0) {
        // Normal refraction case
        currentRay = ray(hitPoint - N * RAY_EPSILON, T, currentRay.getAtten(), ray::REFRACTION);

        // Calculate transmittance (Beer's law) if we're exiting a medium
        glm::dvec3 transmittance(1.0);
        if (!entering) {
          double d = glm::distance(currentRay.getPosition(), hitPoint);
          // Use absorption coefficient to limit extreme darkening
          glm::dvec3 absorb = glm::min(material.kt(i), glm::dvec3(5.0));
          transmittance = glm::exp(-absorb * d);
        }

        throughput *= transmittance * material.kt(i) / probRefract;
        specularBounce = true;
      }
      else {
        // Total internal reflection case
        glm::dvec3 R = glm::normalize(glm::reflect(D, N));
        currentRay = ray(hitPoint + N * RAY_EPSILON, R, currentRay.getAtten(), ray::REFLECTION);
        throughput *= material.kr(i) / (probReflect > 0.0 ? probReflect : 1.0);
        specularBounce = true;
      }
    }
    else if (random < (accumulatedProb += probSpecular) && ksLength > 0.0) {
      // Glossy specular reflection
      double roughness = 1.0 - glm::pow(material.shininess(i) / 128.0, 0.5);
      roughness = glm::clamp(roughness, 0.01, 0.99);
      
      glm::dvec3 V = -D;  // View direction
      glm::dvec3 L = sampleGGX(N, V, roughness);
      
      if (glm::dot(L, N) > 0.0) {
        currentRay = ray(hitPoint + N * RAY_EPSILON, L, currentRay.getAtten(), ray::REFLECTION);
        double NdotL = glm::max(glm::dot(N, L), 0.0);
        throughput *= material.ks(i) * NdotL / probSpecular;
        specularBounce = true;
      } else {
        break; // Invalid direction
      }
    }
    else {
      // Diffuse reflection
      // Create orthonormal basis around normal
      glm::dvec3 w = N;
      glm::dvec3 u = glm::normalize(glm::cross((fabs(w.x) > 0.1 ? glm::dvec3(0, 1, 0) : glm::dvec3(1, 0, 0)), w));
      glm::dvec3 v = glm::cross(w, u);
      
      // Cosine-weighted sample on hemisphere
      double r1 = static_cast<double>(rand()) / RAND_MAX;
      double r2 = static_cast<double>(rand()) / RAND_MAX;
      double phi = 2.0 * M_PI * r1;
      double theta = sqrt(r2);
      double x = theta * cos(phi);
      double y = theta * sin(phi);
      double z = sqrt(1.0 - theta * theta);
      
      // Convert to world space
      glm::dvec3 sampledDir = glm::normalize(u * x + v * y + w * z);
      currentRay = ray(hitPoint + N * RAY_EPSILON, sampledDir, currentRay.getAtten(), ray::VISIBILITY);
      
      throughput *= material.kd(i) / probDiffuse;
    }

    // Russian roulette termination with increased survival probability
    if (depth > 2) {
      double luminance = 0.3 * throughput.r + 0.6 * throughput.g + 0.1 * throughput.b;
      double continueProbability = std::min(0.98, luminance);
      
      // Ensure some minimum chance to continue for dark paths
      continueProbability = std::max(continueProbability, 0.2);
      
      if (static_cast<double>(rand()) / RAND_MAX > continueProbability) {
        break;
      }
      throughput /= continueProbability;
    }
  }

  return glm::clamp(L, 0.0, 1.0);
}
RayTracer::RayTracer()
    : scene(nullptr), buffer(0), thresh(0), buffer_width(0), buffer_height(0),
      m_bBufferReady(false) {
}

RayTracer::~RayTracer() {}

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
    // .ray Parsing Path
    // Call this with 'true' for debug output from the tokenizer
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
      string msg("Invalid JSON encountered ");
      msg.append(je.what());
      traceUI->alert(msg);
      return false;
    }
  }

  if (!sceneLoaded())
    return false;

  return true;
}

void RayTracer::traceSetup(int w, int h) {
  size_t newBufferSize = w * h * 3;
  if (newBufferSize != buffer.size()) {
    bufferSize = newBufferSize;
    buffer.resize(bufferSize);
    
    // Reset sample count for each pixel when buffer size changes
    samplesPerPixel.resize(w * h);
    std::fill(samplesPerPixel.begin(), samplesPerPixel.end(), 0);
  }
  buffer_width = w;
  buffer_height = h;
  std::fill(buffer.begin(), buffer.end(), 0);
  m_bBufferReady = true;

  /*
   * Sync with TraceUI
   */
  threads = traceUI->getThreads();
  block_size = traceUI->getBlockSize();
  thresh = traceUI->getThreshold();
  samples = traceUI->getSuperSamples();
  aaThresh = traceUI->getAaThreshold();

  bvhMaxDepth = traceUI->getMaxDepth();
  bvhTargetLeafSize = traceUI->getLeafSize();

  int sampleDimension = std::max(1, static_cast<int>(std::sqrt(samples)));
  generateStratifiedSamples(sampleDimension);

  if (traceUI->bvhSwitch()) {
    scene->buildBVH(bvhMaxDepth, bvhTargetLeafSize);
  } else {
    scene->clearBVH();
  }
}

glm::dvec3 RayTracer::tracePixel(int i, int j) {
  glm::dvec3 col(0, 0, 0);

  if (!sceneLoaded())
    return col;

  double x = double(i) / double(buffer_width);
  double y = double(j) / double(buffer_height);

  // Get pixel index
  int pixelIndex = i + j * buffer_width;
  
  // Get existing color for progressive rendering
  unsigned char *pixel = buffer.data() + pixelIndex * 3;
  glm::dvec3 existingColor(pixel[0] / 255.0, pixel[1] / 255.0, pixel[2] / 255.0);
  
  // Compute the new sample
  col = trace(x, y);
  
  // For progressive rendering, blend with existing color
  int currentSamples = samplesPerPixel[pixelIndex];
  
  if (currentSamples > 0) {
    // Blend new sample with existing average
    glm::dvec3 blendedColor = (existingColor * static_cast<double>(currentSamples) + col) / 
                              static_cast<double>(currentSamples + 1);
    
    pixel[0] = (int)(255.0 * blendedColor[0]);
    pixel[1] = (int)(255.0 * blendedColor[1]);
    pixel[2] = (int)(255.0 * blendedColor[2]);
    
    col = blendedColor;
  } else {
    // First sample
    pixel[0] = (int)(255.0 * col[0]);
    pixel[1] = (int)(255.0 * col[1]);
    pixel[2] = (int)(255.0 * col[2]);
  }
  
  // Increment sample count for this pixel
  samplesPerPixel[pixelIndex]++;
  
  return col;
}

void RayTracer::workerThread(int threadId) {
  while (true) {
      Pixel pixel(0, 0, nullptr);
      
      // Get next pixel from queue
      {
          std::lock_guard<std::mutex> lock(bufferMutex);
          if (pixelQueue.empty()) {
              threadDone[threadId] = true;
              return;
          }
          pixel = pixelQueue.front();
          pixelQueue.pop();
      }
      
      // Process the pixel
      glm::dvec3 color = tracePixel(pixel.ix, pixel.jy);
      
      // Update the buffer
      {
          std::lock_guard<std::mutex> lock(bufferMutex);
          pixel.value[0] = (unsigned char)(255.0 * color[0]);
          pixel.value[1] = (unsigned char)(255.0 * color[1]);
          pixel.value[2] = (unsigned char)(255.0 * color[2]);
      }
  }
}

/*
 * RayTracer::traceImage
 *
 *	Trace the image and store the pixel data in RayTracer::buffer.
 *
 *	Arguments:
 *		w:	width of the image buffer
 *		h:	height of the image buffer
 *
 */
void RayTracer::traceImage(int w, int h) {
  traceSetup(w, h);
  
  // Clear any existing threads
  waitRender();
  workerThreads.clear();
  threadDone.clear();
  
  // Initialize thread status tracking
  threadDone.resize(threads, false);
  
  // Fill the pixel queue
  while (!pixelQueue.empty()) pixelQueue.pop();
  for (int j = 0; j < h; ++j) {
      for (int i = 0; i < w; ++i) {
          unsigned char* pixel = buffer.data() + (i + j * w) * 3;
          pixelQueue.push(Pixel(i, j, pixel));
      }
  }
  
  // Start the worker threads
  for (unsigned int i = 0; i < threads; ++i) {
      workerThreads.push_back(std::thread(&RayTracer::workerThread, this, i));
  }
  
  // Return immediately - rendering continues asynchronously
}

int RayTracer::aaImage() {
  return 0;
}

bool RayTracer::checkRender() {
  if (workerThreads.empty()) {
      return true;
  }
  
  for (bool done : threadDone) {
      if (!done) return false;
  }
  return true;
}

void RayTracer::waitRender() {
  for (auto& thread : workerThreads) {
      if (thread.joinable()) {
          thread.join();
      }
  }
  workerThreads.clear();
  threadDone.clear();
}


glm::dvec3 RayTracer::getPixel(int i, int j) {
  unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;
  return glm::dvec3((double)pixel[0] / 255.0, (double)pixel[1] / 255.0,
                    (double)pixel[2] / 255.0);
}

void RayTracer::setPixel(int i, int j, glm::dvec3 color) {
  unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;

  pixel[0] = (int)(255.0 * color[0]);
  pixel[1] = (int)(255.0 * color[1]);
  pixel[2] = (int)(255.0 * color[2]);
}

// Add these implementation methods

void RayTracer::generateStratifiedSamples(int samplesPerDimension) {
  int totalSamples = samplesPerDimension * samplesPerDimension;
  stratifiedSamples.resize(totalSamples);
  
  // Generate stratified samples
  for (int y = 0; y < samplesPerDimension; ++y) {
    for (int x = 0; x < samplesPerDimension; ++x) {
      double jitterX = static_cast<double>(rand()) / RAND_MAX;
      double jitterY = static_cast<double>(rand()) / RAND_MAX;
      
      stratifiedSamples[y * samplesPerDimension + x] = glm::dvec2(
        (x + jitterX) / samplesPerDimension,
        (y + jitterY) / samplesPerDimension
      );
    }
  }
  
  // Shuffle the samples for better distribution
  for (int i = totalSamples - 1; i > 0; --i) {
    int j = rand() % (i + 1);
    std::swap(stratifiedSamples[i], stratifiedSamples[j]);
  }
  
  currentSampleIndex = 0;
}

glm::dvec2 RayTracer::getNextSample2D() {
  // If we've used all samples, regenerate
  if (currentSampleIndex >= stratifiedSamples.size()) {
    generateStratifiedSamples(static_cast<int>(sqrt(stratifiedSamples.size())));
  }
  
  return stratifiedSamples[currentSampleIndex++];
}
