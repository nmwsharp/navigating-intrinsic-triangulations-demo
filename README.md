# navigating-intrinsic-triangulations-demo
Demo code for "Navigating Intrinsic Triangulations". Sharp, Soliman, and Crane. 2019 

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

| flag | purpose | arguments |
| :------------- |:------------- | :-----|
| `--noGUI` | Do not show the GUI, just process options and exit | |
| `--flipDelaunay` | Flip edges to make the mesh intrinsic Delaunay | |
| `--refineDelaunay` | Refine and flip edges to make the mesh intrinsic Delaunay and satisfy angle/size bounds | |
| `--refineAngle` | Minimum angle threshold (in degrees). | the angle, default: `25.` |
| `--refineSizeCircum` | Maximum triangle size, set by specifying the circumradius. | the circumradius, default: `inf` |
| `--refineMaxInsertions` | Maximum number of insertions during refinement. Use 0 for no max, or negative values to scale by number of vertices. | the count, default: `-10` (= 10 * nVerts) |
| `--outputPrefix` |  Prefix to prepend to all output file paths | the prefix, default: `intrinsic_`|
| `--intrinsicFaces` | Write the face information for the intrinsic triangulation. These are two dense `Fx3` matrices, giving the indices of the vertices for each face, and the length of the edge from `i` to `(i+1)%3`'th adjacent vertex. Names: `faceInds.dmat`, `faceLengths.dmat` | |
| `--vertexPositions` | Write the vertex positions for the intrinsic triangulation. A dense `Vx3` matrix of 3D coordinates. Name: `vertexPositions.dmat` | |
| `--laplaceMat` | Write the Laplace-Beltrami matrix for the triangulation. A sparse `VxV` matrix, holding the _weak_ Laplace matrix (that is, does not include mass matrix). Name: `laplace.spmat` | |
| `--interpolateMat` | Write the matrix which expresses data on the intrinsic vertices as a linear combination of the input vertices. A sparse, `VxV` matrix where each row has up to 3 nonzero entries that sum to 1. The column indices will always be in the first `V_0` vertices, the original input vertices. Name: `interpolate.mat`| |

Notice that the vertices are indexed such that original input vertices appear first.

#### Output formats

Dense matrices are output as an ASCII file where each line is a row of the matrix, which entries separated by spaces. The first line is a comment prefixed by `#`, giving the number of rows and columns in the matrix. Such files can be automatically loaded in many environments (e.g numpy and matlab).

Sparse matrices are output as an ASCII file where each line one entry in the matrix, giving the row, column, and value. The row and column indices are **1-indexed** to make matlab happy. The first line is a comment prefixed by `#`, giving the number of rows and columns in the matrix. These files can be automatically loaded in matlab ([see here](https://www.mathworks.com/help/matlab/ref/spconvert.html)). Writing parsers in other environments should be straightforward.
