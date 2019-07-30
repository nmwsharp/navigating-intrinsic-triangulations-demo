#include "geometrycentral/surface/halfedge_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/signpost_intrinsic_triangulation.h"
#include "geometrycentral/surface/surface_centers.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "args/args.hxx"
#include "imgui.h"

#include <sstream>

using namespace geometrycentral;
using namespace geometrycentral::surface;

// == Geometry-central data
std::unique_ptr<HalfedgeMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geometry;

std::unique_ptr<SignpostIntrinsicTriangulation> signpostTri;

// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh* psMesh;

// Parameters
float refineToSize = -1;
float refineDegreeThresh = 29.9;
bool useRefineSizeThresh = false;
bool useInsertionsMax;
int insertionsMax = -2;

// Mesh stats
bool signpostIsDelaunay = true;
float signpostMinAngleDeg = 0.;

void updateTriagulationViz() {
  // Update stats
  signpostIsDelaunay = signpostTri->isDelaunay();
  signpostMinAngleDeg = signpostTri->minAngleDegrees();


  // Get the edge traces
  EdgeData<std::vector<SurfacePoint>> traces = signpostTri->traceEdges();

  // Convert to 3D positions
  std::vector<std::vector<Vector3>> traces3D(traces.size());
  size_t i = 0;
  for (Edge e : signpostTri->mesh.edges()) {
    for (SurfacePoint& p : traces[e]) {
      traces3D[i].push_back(p.interpolate(geometry->inputVertexPositions));
    }
    i++;
  }

  // Register with polyscope
  auto graphQ = polyscope::getSurfaceMesh()->addSurfaceGraphQuantity("intrinsic edges", traces3D);
  graphQ->setEnabled(true);
}

void resetTriangulation() {
  signpostTri.reset(new SignpostIntrinsicTriangulation(*geometry));
  updateTriagulationViz();
}


void flipDelaunayTriangulation() {
  signpostTri->flipToDelaunay();

  if (!signpostTri->isDelaunay()) {
    polyscope::warning("woah, failed to make mesh Delaunay with flips");
  }

  updateTriagulationViz();
}

void refineDelaunayTriangulation() {

  // Manage optional parameters
  double sizeParam = useRefineSizeThresh ? refineToSize : std::numeric_limits<double>::infinity();
  size_t maxInsertions = useInsertionsMax ? insertionsMax : INVALID_IND;

  signpostTri->delaunyRefine(refineDegreeThresh, sizeParam, maxInsertions);

  if (!signpostTri->isDelaunay()) {
    polyscope::warning("Failed to make mesh Delaunay with flips & refinement. Bug Nick to finish porting implementation.");
  }

  updateTriagulationViz();
}

void myCallback() {

  ImGui::PushItemWidth(100);

  ImGui::TextUnformatted("Intrinsic triangulation:");
  ImGui::Text("  nVertices = %lu  nFaces = %lu", signpostTri->mesh.nVertices(), signpostTri->mesh.nFaces());
  if (signpostIsDelaunay) {
    ImGui::Text("  is Delaunay: yes  min angle = %.2f degrees", signpostMinAngleDeg);
  } else {
    ImGui::Text("  is Delaunay: no   min angle = %.2f degrees", signpostMinAngleDeg);
  }

  if (ImGui::Button("reset triangulation")) {
    resetTriangulation();
  }

  if (ImGui::TreeNode("Delaunay flipping")) {
    if (ImGui::Button("flip to Delaunay")) {
      flipDelaunayTriangulation();
    }
    ImGui::TreePop();
  }


  if (ImGui::TreeNode("Delaunay refinement")) {
    ImGui::InputFloat("degree threshold", &refineDegreeThresh);

    ImGui::Checkbox("refine large triangles", &useRefineSizeThresh);
    if (useRefineSizeThresh) {
      ImGui::InputFloat("size threshold (circumradius)", &refineToSize);
    }

    ImGui::Checkbox("limit number of insertions", &useInsertionsMax);
    if (useInsertionsMax) {
      ImGui::InputInt("num insertions", &insertionsMax);
    }

    if (ImGui::Button("Delaunay refine")) {
      refineDelaunayTriangulation();
    }
    ImGui::TreePop();
  }

  ImGui::PopItemWidth();
}

int main(int argc, char** argv) {

  // Configure the argument parser
  args::ArgumentParser parser("A demo of Naavigating Intrinsic Triangulations");
  args::Positional<std::string> inputFilename(parser, "mesh", "A .obj or .ply mesh file.");

  // Parse args
  try {
    parser.ParseCLI(argc, argv);
  } catch (args::Help) {
    std::cout << parser;
    return 0;
  } catch (args::ParseError e) {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 1;
  }

  // Make sure a mesh name was given
  if (!inputFilename) {
    std::cerr << "Please specify a mesh file as argument" << std::endl;
    return EXIT_FAILURE;
  }

  // Initialize polyscope
  polyscope::init();

  // Set the callback function
  polyscope::state::userCallback = myCallback;

  // Load mesh
  std::tie(mesh, geometry) = loadMesh(args::get(inputFilename));

  // Register the mesh with polyscope
  psMesh = polyscope::registerSurfaceMesh(polyscope::guessNiceNameFromPath(args::get(inputFilename)),
                                          geometry->inputVertexPositions, mesh->getFaceVertexList(),
                                          polyscopePermutations(*mesh));


  // Set vertex tangent spaces
  geometry->requireVertexTangentBasis();
  VertexData<Vector3> vBasisX(*mesh);
  for (Vertex v : mesh->vertices()) {
    vBasisX[v] = geometry->vertexTangentBasis[v][0];
  }
  polyscope::getSurfaceMesh()->setVertexTangentBasisX(vBasisX);

  // Set face tangent spaces
  geometry->requireFaceTangentBasis();
  FaceData<Vector3> fBasisX(*mesh);
  for (Face f : mesh->faces()) {
    fBasisX[f] = geometry->faceTangentBasis[f][0];
  }
  polyscope::getSurfaceMesh()->setFaceTangentBasisX(fBasisX);

  // Nice defaults
  psMesh->edgeWidth = 1.0;

  // Set initial values for parameters
  double surfaceArea = 0.;
  geometry->requireFaceAreas();
  for (Face f : mesh->faces()) {
    surfaceArea += geometry->faceAreas[f];
  }
  double meanFaceArea = surfaceArea / mesh->nFaces();
  refineToSize = std::sqrt(meanFaceArea);
  insertionsMax = 5 * mesh->nVertices();
  useInsertionsMax = true;


  // Initialize triangulation
  resetTriangulation();

  // Give control to the polyscope gui
  polyscope::show();

  return EXIT_SUCCESS;
}
