//
// Created by ery on 2020/05/18.
//

#include "ViewerViz.hpp"

#include <thread>

vslam::viewer::ViewerViz::ViewerViz(const std::string& window_name,
                                    int32_t window_width,
                                    int32_t window_height)
    : window_name_(window_name),
      window_size_width_(window_width),
      window_size_height_(window_height),
      threadsafe_primitive_queue_(window_name_),
      is_shutdown_(false) {
  InitializeViewer(window_name_, window_size_width_, window_size_height_);
}

void vslam::viewer::ViewerViz::LaunchViewer(bool is_block) {
  // use the context in a separate rendering thread
  render_loop_ = std::thread(&ViewerViz::ViewerLoop, this);
  if (is_block) {
    render_loop_.join();
  }
}

void vslam::viewer::ViewerViz::InitializeViewer(const std::string& window_name,
                                                int32_t window_width,
                                                int32_t window_height) {
  window_ = cv::viz::Viz3d(window_name);
  //    window_.setBackgroundColor(cv::viz::Color::white());
  window_.setBackgroundColor(cv::viz::Color::gray(), cv::viz::Color::black());
  //  window_.addLight(cv::Vec3d(100,100,100),
  //                   cv::Vec3d(0,0,0),
  //                   cv::viz::Color::white(),
  //                   cv::viz::Color::white(),
  //                   cv::viz::Color::white(),
  //                   cv::viz::Color::white()
  //                   );

  window_.addLight(cv::Vec3d(0, 0, 100),
                   cv::Vec3d(0, 0, 0),
                   cv::viz::Color::white(),
                   cv::viz::Color::white(),
                   cv::viz::Color::white(),
                   cv::viz::Color::white());
  window_.addLight(cv::Vec3d(100, 100, 100),
                   cv::Vec3d(0, 0, 0),
                   cv::viz::Color::white(),
                   cv::viz::Color::white(),
                   cv::viz::Color::white(),
                   cv::viz::Color::white());
  window_.addLight(cv::Vec3d(0, 0, -100),
                   cv::Vec3d(0, 0, 0),
                   cv::viz::Color::white(),
                   cv::viz::Color::white(),
                   cv::viz::Color::white(),
                   cv::viz::Color::white());
  window_.addLight(cv::Vec3d(-100, -100, -100),
                   cv::Vec3d(0, 0, 0),
                   cv::viz::Color::white(),
                   cv::viz::Color::white(),
                   cv::viz::Color::white(),
                   cv::viz::Color::white());

  // Feed primitive database from queue
  spdlog::info("Pop primitive queue.");
  while (true) {
    auto widget_ptr = threadsafe_primitive_queue_.pop();
    if (widget_ptr == nullptr) {
      spdlog::info("Queue empty.");
      break;
    }
    int32_t count = 0;
    for (const auto& w : (*widget_ptr)->GetWidget()) {
      primitive_database_[(*widget_ptr)->GetTag() + std::to_string(count++)] =
          PrimitiveInfo{w, (*widget_ptr)->GetPose()};
    }
    //    primitive_database_[(*widget_ptr)->GetTag()] =
    //        PrimitiveInfo{(*widget_ptr)->GetWidget(),
    //        (*widget_ptr)->GetPose()};
    //    primitive_database_[widget_ptr->GetTag()] =
    //    PrimitiveInfo{widget_ptr->GetWidget(), widget_ptr->GetPose()};
  }
}
void vslam::viewer::ViewerViz::ViewerLoop() {
  spdlog::info("Launch drawing thread.");
  while (!is_shutdown_) {
    //    spdlog::info("Start drawing.");
    //    window_.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

    // Feed primitive database from queue
    //    spdlog::info("Pop primitive queue.");
    while (true) {
      auto widget_ptr = threadsafe_primitive_queue_.pop();
      if (widget_ptr == nullptr) {
        //        spdlog::info("Queue empty.");
        break;
      }
      int32_t count = 0;
      for (const auto& w : (*widget_ptr)->GetWidget()) {
        primitive_database_[(*widget_ptr)->GetTag() + std::to_string(count++)] =
            PrimitiveInfo{w, (*widget_ptr)->GetPose()};
      }
      //      primitive_database_[(*widget_ptr)->GetTag()] =
      //          PrimitiveInfo{(*widget_ptr)->GetWidget(),
      //          (*widget_ptr)->GetPose()};
      //      primitive_database_[widget_ptr->GetTag()] =
      //      PrimitiveInfo{widget_ptr->GetWidget(), widget_ptr->GetPose()};
    }

    // Render primitives
    for (const auto& [tag, widget] : primitive_database_) {
      window_.showWidget(tag, widget.widget_, widget.pose_);
    }
    window_.spinOnce(10, true);
  }
  spdlog::info("Shutdown drawing thread.");
}

void vslam::viewer::ViewerViz::PushPrimitive(
    const PrimitiveBase& input_primitive) {
  threadsafe_primitive_queue_.push(
      std::shared_ptr<PrimitiveBase>(input_primitive.Clone()));
}