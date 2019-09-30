# navigating-intrinsic-triangulations-demo
Demo code for "Navigating Intrinsic Triangulations". Sharp, Soliman, and Crane. 2019 

**This code is a preview! It is still under construction and there is more to come!**

Our core signpost data structure is implemented in [geometry-central](http://geometry-central.net). This is a simple appliction which loads a mesh, computes an intrinsic triangulation, and visualizes its edges.  Additionally, the code can be invoked from the command line to output data about the intrinsic triangulation in easily-parseable formats.  In this library, the resulting `SignpostIntrinsicTriangulation` class can be used with all geometry routines in geometry-central. 

**Note:** This is not the version of the code which was used to generate the results in the paper; it has been simplified to make it easier to use. If you want to generate precise comparisons against the paper, please contact the authors.

### Building and running

On unix-like machines, use:
```
git clone --recurse-submodules https://github.com/nmwsharp/navigating-intrinsic-triangulations-demo.git
cd navigating-intrinsic-triangulations-demo
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
./bin/int_tri /path/to/your/mesh.obj
```

The codebase also builds on Visual Studio 2017 & 2019 (at least), by using CMake to generate a Visual Studio solution file.

Running the program open a UI window showing your mesh. The intrinsic triangulation is denoted by the colored edge tubes, whose thickness can be adjusted in the settings panel on the left.

The command window in the upper right can be used to flip the intrinsic triangulation to Delaunay, as well as perform Delaunay refinement. It also has options for outputting to file (see the command line documentation below).

### Command line interface

#### Output formats

Dense matrices are output as an ASCII file where each line is a row of the matrix, which entries separated by spaces. The first line is a comment prefixed by `#`, giving the number of rows and columns in the matrix. Such files can be automatically loaded in many environments (e.g numpy and matlab).

Sparse matrices are output as an ASCII file where each line one entry in the matrix, giving the row, column, and value. The row and column indices are **1-indexed** to make matlab happy. The first line is a comment prefixed by `#`, giving the number of rows and columns in the matrix. These files can be automatically loaded in matlab ([see here](https://www.mathworks.com/help/matlab/ref/spconvert.html)). Writing parsers in other environments should be straightforward.
