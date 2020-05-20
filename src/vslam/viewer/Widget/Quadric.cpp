//
// Created by ery on 2020/05/19.
//

#include "Quadric.hpp"

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkContourFilter.h>
#include <vtkIdList.h>
#include <vtkImageData.h>
#include <vtkOutlineFilter.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProp.h>
#include <vtkProperty.h>
#include <vtkQuadric.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSampleFunction.h>
#include <vtkSmartPointer.h>
#include <vtkSuperquadric.h>
#include <vtkSuperquadricSource.h>
#include <vtkTriangle.h>

using namespace cv;
using namespace vslam::viewer;

WQuadric::WQuadric(const std::array<double, 10>& quadric_coefficients,
                   const viz::Color& color) {
  auto quadric = vtkSmartPointer<vtkQuadric>::New();
  quadric->SetCoefficients(quadric_coefficients[0],
                           quadric_coefficients[1],
                           quadric_coefficients[2],
                           quadric_coefficients[3],
                           quadric_coefficients[4],
                           quadric_coefficients[5],
                           quadric_coefficients[6],
                           quadric_coefficients[7],
                           quadric_coefficients[8],
                           quadric_coefficients[9]);

  auto super_quadric = vtkSmartPointer<vtkSuperquadric>::New();
  super_quadric->SetScale(0.5, 1, 1);

  auto sample = vtkSmartPointer<vtkSampleFunction>::New();
  //  sample->SetImplicitFunction(quadric);
  sample->SetImplicitFunction(quadric);
  //  sample->SetModelBounds(-0.5, 0.5, -0.5, 0.5, -0.5, 0.5);
  //  sample->SetSampleDimensions(40, 40, 40);
  //  sample->ComputeNormalsOff();

  // contour
  auto surface = vtkSmartPointer<vtkContourFilter>::New();
  surface->SetInputConnection(sample->GetOutputPort());
  surface->SetValue(0, 0.0);

  // Create a mapper and an actor
  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(surface->GetOutputPort());
  mapper->ScalarVisibilityOff();
  auto actor = vtkSmartPointer<vtkActor>::New();

  actor->SetMapper(mapper);
  //  actor->GetProperty()->EdgeVisibilityOn(); // Wire
  //  frameを表示させたいときはOnにする
  cv::viz::WidgetAccessor::setProp(*this, actor);

  setColor(color);
}

WQuadric::WQuadric(const vslam::Vec3_t& scale, const cv::viz::Color& color) {
  double epsilon = 1e-3;
  vslam::Vec3_t scale_eps{
      scale[0] + epsilon, scale[1] + epsilon, scale[2] + epsilon};
  auto super_quadric = vtkSmartPointer<vtkSuperquadric>::New();
  super_quadric->SetSize(scale_eps[0]);
  super_quadric->SetScale(
      1.0, scale_eps[1] / scale_eps[0], scale_eps[2] / scale_eps[0]);
  super_quadric->SetPhiRoundness(1);
  super_quadric->SetThetaRoundness(1);

  auto sample = vtkSmartPointer<vtkSampleFunction>::New();
  sample->SetImplicitFunction(super_quadric);

  int32_t sample_number = 10;
  sample->SetModelBounds(-scale_eps[0],
                         scale_eps[0],
                         -scale_eps[1],
                         scale_eps[1],
                         -scale_eps[2],
                         scale_eps[2]);
  sample->SetSampleDimensions(sample_number, sample_number, sample_number);
  sample->ComputeNormalsOff();

  // contour
  auto surface = vtkSmartPointer<vtkContourFilter>::New();
  surface->SetInputConnection(sample->GetOutputPort());
  surface->SetValue(0, 0.0);

  // Create a mapper and an actor
  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(surface->GetOutputPort());
  mapper->ScalarVisibilityOff();
  auto actor = vtkSmartPointer<vtkActor>::New();

  actor->SetMapper(mapper);
  //   actor->GetProperty()->EdgeVisibilityOn(); // Wire
  //  frameを表示させたいときはOnにする
  cv::viz::WidgetAccessor::setProp(*this, actor);

  setColor(color);
}

WTriangleSample::WTriangleSample(const cv::Point3f& pt1,
                                 const cv::Point3f& pt2,
                                 const cv::Point3f& pt3,
                                 const cv::viz::Color& color) {
  // Create a triangle
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(pt1.x, pt1.y, pt1.z);
  points->InsertNextPoint(pt2.x, pt2.y, pt2.z);
  points->InsertNextPoint(pt3.x, pt3.y, pt3.z);
  vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
  triangle->GetPointIds()->SetId(0, 0);
  triangle->GetPointIds()->SetId(1, 1);
  triangle->GetPointIds()->SetId(2, 2);
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
  cells->InsertNextCell(triangle);
  // Create a polydata object
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  // Add the geometry and topology to the polydata
  polyData->SetPoints(points);
  polyData->SetPolys(cells);
  // Create mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
  mapper->SetInput(polyData);
#else
  mapper->SetInputData(polyData);
#endif
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  // Store this actor in the widget in order that visualizer can access it
  cv::viz::WidgetAccessor::setProp(*this, actor);
  // Set the color of the widget. This has to be called after WidgetAccessor.
  setColor(color);
}
