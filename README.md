# navigating-intrinsic-triangulations-demo
Demo code for "Navigating Intrinsic Triangulations". Sharp, Soliman, and Crane. 2019 

**This code is a preview! It is still under construction and there is more to come!**

Our core signpost data structure is implemented in [geometry-central](http://geometry-central.net). This is a simple appliction which loads a mesh, computes an intrinsic triangulation, and visualizes its edges. Notice that the resulting `SignpostIntrinsicTriangulation` class can be used with all geometry routines in geometry-central.


### Building and running

```
git clone --recurse-submodules https://github.com/nmwsharp/navigating-intrinsic-triangulations-demo.git
cd navigating-intrinsic-triangulations-demo
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
./bin/int_tri /path/to/your/mesh.obj
```

This should open a UI window showing your mesh. The intrinsic triangulation is denoted by the colored edge tubes, whose thickness can be adjusted in the settings panel on the left.

The command window in the upper right can be used to flip the intrinsic triangulation to Delaunay, as well as perform Delaunay refinement.
