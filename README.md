# Names
Vinson Zheng
UT EID: vz2275

Zhongyan He
UT EID: zh5555

## Running Reference Binaries on MacOS arm64 Sonoma
For the fltk dependency:
```
brew install fltk@1.3
brew link fltk@1.3
cd /opt/homebrew/opt/
cp -r fltk@1.3 fltk
```

## Submission
```
git archive HEAD -o ${PWD##*/}.zip --prefix ray/
```

This passes sancheck

## Partial Credit for Milestone 1
Our refractions, in particular handling translucent/transparent objects using kt^d, especially for scenes like sphere_refract3 and sphere_refract4 don't seem to be getting the inside refractive rays or some other issue. This is also apparent in cyl_box_transp_shadow.  
Edit: This refraction issue has been fixed.

## Extra Credit  
We added 7 custom scenes for this milestone, which are all in the created-scenes folder.

Support for translucent emissive objects has also been added, which can be tested with the translucent-emissive.json scene in created-scenes.  

Adaptive termination for secondary rays has been added as well, which is controlled by the threshold slider in the GUI. A lower threshold will cause the ray tracer to consider more rays even if their contributions are small. A higher threshold will not consider as many rays and thus may be missing finer details, but will run faster as a result.

We also implemented stochastic (jittered) supersampling for our anti-aliasing.

We also implmented normal mapping which can be tested with the spiral_box_normal.json and spiral_box.json custom scenes.
We used https://cpetry.github.io/NormalMap-Online/ to create normal maps.
We used rgb.py to make sure our normal maps only had RGB using the pillow library.
To add normal mapping to a scene, we added a new parsable key to the json called "normalMap" within the "material" section.
