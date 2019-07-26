#include "geometrycentral/surface/halfedge_mesh.h"
#include "geometrycentral/surface/heat_method_distance.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_centers.h"
#include "geometrycentral/surface/vector_heat_method.h"
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

// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh* psMesh;


void updateTriagulationViz() {}

void resetTriangulation() { updateTriagulationViz(); }


void myCallback() {

  ImGui::PushItemWidth(300);
  if (ImGui::Button("reset triangulation")) {
    resetTriangulation();
  }
  ImGui::PopItemWidth();
}

int main(int argc, char** argv) {

  // Configure the argument parser
  args::ArgumentParser parser("A demo of Naavigating Intrinsic Triangulations");
  args::Positional<std::string> inputFilename(parser, "mesh", "A mesh file.");

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


  // Give control to the polyscope gui
  polyscope::show();

  return EXIT_SUCCESS;
}
