// The main ray tracer.

#pragma warning(disable : 4786)

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
  // Clear out the ray cache in the scene for debugging purposes,
  if (TraceUI::m_debug) {
    scene->clearIntersectCache();
  }

  ray r(glm::dvec3(0, 0, 0), glm::dvec3(0, 0, 0), glm::dvec3(1, 1, 1),
        ray::VISIBILITY);
  scene->getCamera().rayThrough(x, y, r);
  double dummy;
  glm::dvec3 ret =
      traceRay(r, glm::dvec3(1.0, 1.0, 1.0), traceUI->getDepth(), dummy);
  ret = glm::clamp(ret, 0.0, 1.0);
  return ret;
}

glm::dvec3 RayTracer::tracePixel(int i, int j) {
  glm::dvec3 col(0, 0, 0);

  if (!sceneLoaded())
    return col;

  double x = double(i) / double(buffer_width);
  double y = double(j) / double(buffer_height);

  unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;
  col = trace(x, y);

  pixel[0] = (int)(255.0 * col[0]);
  pixel[1] = (int)(255.0 * col[1]);
  pixel[2] = (int)(255.0 * col[2]);
  return col;
}

#define VERBOSE 0

// Do recursive ray tracing! You'll want to insert a lot of code here (or places
// called from here) to handle reflection, refraction, etc etc.
glm::dvec3 RayTracer::traceRay(ray &r, const glm::dvec3 &thresh, int depth,
                               double &t) {
  isect i;
  glm::dvec3 colorC;
#if VERBOSE
  std::cerr << "== current depth: " << depth << std::endl;
#endif

  if (scene->intersect(r, i)) {
    // YOUR CODE HERE

    // An intersection occurred!  We've got work to do. For now, this code gets
    // the material for the surface that was intersected, and asks that material
    // to provide a color for the ray.

    // This is a great place to insert code for recursive ray tracing. Instead
    // of just returning the result of shade(), add some more steps: add in the
    // contributions from reflected and refracted rays.

    const Material &m = i.getMaterial();
    colorC = m.shade(scene.get(), r, i);
    t = i.getT();

    if (depth > 0) {
      // reflection
      glm::dvec3 D = glm::normalize(r.getDirection());
      glm::dvec3 N = glm::normalize(i.getN());
      glm::dvec3 R = glm::normalize(glm::reflect(D, N));
      glm::dvec3 hitPoint = r.at(i);
      bool entering = (glm::dot(D, N) < 0.0);

      if (!entering) {
        N = -N;
      }
      glm::dvec3 offsetHitPoint = hitPoint + N * RAY_EPSILON;

      // if (debugMode) {
      //   cout << m.Refl() << endl;
      //   cout << m.Trans() << endl;
      //   cout << "hitPoint: " << hitPoint << endl;
      //   cout << "D: " << D << endl;
      //   cout << "N: " << N << endl;
      //   cout << "R: " << R << endl;
      // }

      if (m.Refl()) {
        ray reflectedRay(offsetHitPoint, R, r.getAtten(), ray::REFLECTION);
        glm::dvec3 transmittance = glm::dvec3(1.0, 1.0, 1.0);
        if (!entering && m.Trans()) {
          double d = glm::max(glm::distance(r.getPosition() + D * RAY_EPSILON, r.at(i.getT())), 0.0);
          transmittance = glm::pow(m.kt(i), glm::dvec3(d));
        }
        colorC += transmittance * m.kr(i) * traceRay(reflectedRay, thresh, depth - 1, t);
      }

      // refraction
      if (m.Trans()) {
        double n1 = 1.0;
        double n2 = m.index(i);

        // If we're inside the refracting material
        if (!entering) {
          swap(n1, n2);
        }

        double eta = n1 / n2;
        glm::dvec3 T = glm::normalize(glm::refract(D, N, eta));

        // if (debugMode) {
        //   cout << "n1: " << n1 << endl;
        //   cout << "n2: " << n2 << endl;
        //   cout << "eta: " << eta << endl;
        //   cout << "T: " << T << endl;
        //   cout << endl;
        // }

        if (glm::length(T) > 0.0) {
          // Normal refraction, otherwise TIR and we already shot a reflection ray
          ray refractedRay = ray(hitPoint + D * RAY_EPSILON, T, r.getAtten(), ray::REFRACTION);
          glm::dvec3 transmittance = glm::dvec3(1.0, 1.0, 1.0);
          if (!entering && m.Trans()) {
            double d = glm::max(glm::distance(r.getPosition() + D * RAY_EPSILON, r.at(i.getT())), 0.0);
            transmittance = glm::pow(m.kt(i), glm::dvec3(d)); // Attenuate on exit
          }
          colorC += transmittance * traceRay(refractedRay, thresh, depth - 1, t);;
        }
      }
    }
  } else {
    // No intersection. This ray travels to infinity, so we color
    // it according to the background color, which in this (simple)
    // case is just black.
    //
    // FIXME: Add CubeMap support here.
    // TIPS: CubeMap object can be fetched from
    // traceUI->getCubeMap();
    //       Check traceUI->cubeMap() to see if cubeMap is loaded
    //       and enabled.

    colorC = glm::dvec3(0.0, 0.0, 0.0);
  }
#if VERBOSE
  std::cerr << "== depth: " << depth + 1 << " done, returning: " << colorC
            << std::endl;
#endif
  return colorC;
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

  // YOUR CODE HERE
  // FIXME: Additional initializations
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
  // YOUR CODE HERE
  // FIXME: Implement Anti-aliasing here
  //
  // TIP: samples and aaThresh have been synchronized with TraceUI by
  //      RayTracer::traceSetup() function
  if (samples <= 1) {
      return 0; // No anti-aliasing needed
  }

  // For each pixel
  for (int i = 0; i < buffer_width; i++) {
      for (int j = 0; j < buffer_height; j++) {
          // Get the original pixel color
          glm::dvec3 originalColor = getPixel(i, j);
          
          // Check if anti-aliasing is needed for this pixel by comparing with neighbors
          bool needsAA = false;
          
          // Check neighboring pixels
          for (int di = -1; di <= 1 && !needsAA; di++) {
              for (int dj = -1; dj <= 1; dj++) {
                  // Skip the current pixel
                  if (di == 0 && dj == 0) continue;

                  int ni = i + di;
                  int nj = j + dj;
                  
                  if (ni >= 0 && ni < buffer_width && nj >= 0 && nj < buffer_height) {
                      glm::dvec3 neighborColor = getPixel(ni, nj);
                      
                      // If difference between pixel and neighbor exceeds threshold
                      if (glm::length(originalColor - neighborColor) > aaThresh) {
                          needsAA = true;
                          break;
                      }
                  }
              }
          }

          if (needsAA) {
              glm::dvec3 accumulatedColor(0.0);
              
              // Supersampling grid
              for (int si = 0; si < samples; si++) {
                  for (int sj = 0; sj < samples; sj++) {
                      // Calculate subpixel position.  Note that the offsets need to be within the range [0, 1].
                      // For example if we have 4 samples per pixel (samples=4, pixels per dimension = 2),
                      // then si and sj range from 0 to 3, and the offset becomes (si + 0.5) / 2 to keep the result inside [0,1]
                      double stratumSize = 1.0 / sqrt(samples);
                      double x = (i + (si * stratumSize + stratumSize/2.0)) / double(buffer_width);
                      double y = (j + (sj * stratumSize + stratumSize/2.0)) / double(buffer_height);
                      
                      // Trace ray through subpixel
                      accumulatedColor += trace(x, y);
                  }
              }
              
              // Average the accumulated colors
              glm::dvec3 finalColor = accumulatedColor / (double)(samples * samples);              

              // Set the anti-aliased pixel
              setPixel(i, j, finalColor);
          }
      }
  }

  return 1;
}

bool RayTracer::checkRender() {
  if (workerThreads.empty()) {
      return true;
  }
  
  // Check if all threads are done
  for (bool done : threadDone) {
      if (!done) return false;
  }
  return true;
}

void RayTracer::waitRender() {
  // Join all threads
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
