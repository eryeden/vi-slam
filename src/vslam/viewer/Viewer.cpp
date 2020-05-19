//
// Created by ery on 2020/05/18.
//

#include "Viewer.hpp"

#include <thread>

vslam::viewer::Viewer::Viewer(const std::string& window_name,
                              int32_t window_width,
                              int32_t window_height)
    : window_name_(window_name),
      window_size_width_(window_width),
      window_size_height_(window_height) {
  InitializeViewer(window_name_, window_size_width_, window_size_height_);
}
void vslam::viewer::Viewer::InitializeViewer(const std::string& window_name,
                                             int32_t window_width,
                                             int32_t window_height) {
  // create a window and bind its context to the main thread
  pangolin::CreateWindowAndBind(window_name, window_width, window_height);

  // enable depth
  glEnable(GL_DEPTH_TEST);

  // unset the current context from the main thread
  pangolin::GetBoundWindow()->RemoveCurrent();
}
void vslam::viewer::Viewer::ViewerLoop() {
  // fetch the context and bind it to this thread
  pangolin::BindToContext(window_name_);

  // we manually need to restore the properties of the context
  glEnable(GL_DEPTH_TEST);

  // Define Projection and initial ModelView matrix
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(window_size_width_,
                                 window_size_height_,
                                 420,
                                 420,
                                 320,
                                 240,
                                 0.2,
                                 100),
      pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));

  // Create Interactive View in window
  pangolin::Handler3D handler(s_cam);
  pangolin::View& d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
                              .SetHandler(&handler);

  while (!pangolin::ShouldQuit()) {
    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);

    // Render OpenGL Cube
    pangolin::glDrawColouredCube();

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  // unset the current context from the main thread
  pangolin::GetBoundWindow()->RemoveCurrent();
}

void vslam::viewer::Viewer::LaunchViewer(bool is_block) {
  // use the context in a separate rendering thread
  std::thread render_loop;
  render_loop = std::thread(&Viewer::ViewerLoop, this);
  if (is_block) {
    render_loop.join();
  }
}
