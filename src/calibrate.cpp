#include <iostream>
#include <thread>

#include <tbb/concurrent_queue.h>

#include "calibration/data_reader/dataset_factory.h"
#include "calibration/camera_models/camera_model_factory.h"
#include "calibration/utils/basic_types.h"
#include "calibration/utils/info_metrics.h"
#include "calibration/frontend/Frontend.h"
#include "calibration/backend/SegmentConstruction.h"
#include "calibration/backend/DatabaseThread.h"
#include "calibration/utils/SimParams.h"
#include "calibration/utils/Sampler.h"
#include "calibration/utils/serialization.h"

// Visualization
#include <opencv2/highgui.hpp>

#include <pangolin/display/image_view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/image/image.h>
#include <pangolin/image/image_io.h>
#include <pangolin/image/typed_image.h>
#include <pangolin/pangolin.h>

// GUI functions
void drawScene(pangolin::View& view);
void drawFeatures(pangolin::View& v, size_t cam_id);
void drawPlots();
void render_camera(const Eigen::Matrix4d& T_w_c, double lineWidth,
                          const u_int8_t* color, double sizeFactor);
void getcolor(float p, float np, float& r, float& g, float& b);

// Pangolin constants
constexpr size_t UI_WIDTH = 300;
constexpr size_t IMG_VIEW_WIDTH = 640;
const uint8_t gt_cam_color[3]{0, 0, 0};
const uint8_t est_cam_color[3]{0, 171, 47};
const uint8_t tracked_point_color[3]{0, 50, 255};
const uint8_t stereo_point_color[3]{250, 0, 26};
const uint8_t segment_pose_color[3]{0, 171, 47};
namespace pangolin{
  extern "C" const unsigned char AnonymousPro_ttf[];
}

// Pangolin variables
using Button = pangolin::Var<std::function<void(void)>>; 
pangolin::Var<bool> pause_playback("ui.pause_playback", false, false, true); 
pangolin::Var<bool> show_mask("ui.show_mask", false, false, true);
pangolin::Var<bool> show_features("ui.show_features", true, false, true);
pangolin::Var<bool> show_gt_extrinsics("ui.gt_extrinsics", true, false, true);
pangolin::Var<bool> show_est_extrinsics("ui.est_extrinsics", true, false, true);
pangolin::Var<bool> show_tracked_points("ui.tracked_points", true, false, true);
pangolin::Var<bool> show_stereo_points("ui.stereo_points", true, false, true);
pangolin::Var<bool> show_segment_poses("ui.segment_poses", true, false, true);

pangolin::OpenGlRenderState camera;
gtsam::Pose3 T_wv_est;
gtsam::Pose3 T_wv_noisy_prev;
std::unique_ptr<gtsam::Sampler<gtsam::Pose3>> odometry_sampler;

size_t current_frame_id;

MultiFrameInput::shared_ptr multi_frame;

std::vector<gtsam::Pose3> extrinsics_gt;
std::vector<std::vector<gtsam::Pose3> > segment_poses;
std::vector<gtsam::Point3> tracked_cloud;
std::vector<gtsam::Point3> stereo_cloud;

pangolin::DataLog error_data_log;
pangolin::Plotter* plotter;


// Calibration variables
std::vector<gtsam::PinholeModel::shared_ptr> intrinsics;
std::vector<gtsam::Pose3> extrinsics;
std::vector<std::pair<int, int> > stereo_pairs;

// High-level processing
DataReader::unique_ptr data_reader;
Frontend::unique_ptr frontend;
SegmentConstruction::unique_ptr segment_construction;
DatabaseThread::unique_ptr database_thread;

tbb::concurrent_bounded_queue<MultiFrameInput::shared_ptr> frontend_vis_queue;
tbb::concurrent_bounded_queue<BackendOutput::shared_ptr> backend_vis_queue;

void feedFrames() {
  std::cout << "Starting frame data thread" << std::endl;

  size_t num_frames = data_reader->getNumFrames();
  for (size_t frame_id = 0; frame_id < num_frames - 1; frame_id++) {
  // for (size_t frame_id = 0; frame_id < 20; frame_id++) {
    while (pause_playback) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    current_frame_id = frame_id;
    auto prev_time = std::chrono::steady_clock::now();

    MultiFrameInput::shared_ptr data(new MultiFrameInput);

    data->frame_id_ = frame_id;
    data->timestamp_ = data_reader->getTimestamp(frame_id);

    // Image data
    data->frames_ = data_reader->getFrames(frame_id);
    data->masks_ = data_reader->getMasks(frame_id);

    // std::cout << data->frames_[2]->img_.rows << std::endl;

    // Current estimates
    gtsam::Pose3 T_v1v2_gt = data_reader->getGroundTruthOdom(frame_id);
    gtsam::Pose3 T_v1v2_noisy = odometry_sampler->sample(T_v1v2_gt);
    gtsam::Pose3 T_wv2_noisy = T_wv_noisy_prev * T_v1v2_noisy;
    // T_wv2_noisy = data_reader->getGroundTruthPose(frame_id);
    // std::cout << T_wv2_noisy << " " << data_reader->getGroundTruthPose(frame_id) << std::endl;
    
    // std::cout << data_reader->getTimestamp(frame_id) << std::endl;
    // std::cout << data_reader->getGroundTruthPose(frame_id) << std::endl;

    data->T_wv_est_ = T_wv2_noisy;
    T_wv_noisy_prev = T_wv2_noisy;
    // TODO: If intrinsics get modified, should copy instead of shared ptr?
    data->intrinsics_ = intrinsics;
    data->extrinsics_ = extrinsics;

    // frontend->frame_queue_.push(data);
    if (frontend->frame_queue_.size() < frontend->frame_queue_.capacity()) {
      frontend->frame_queue_.push(data);
    }
    else {
      MultiFrameInput::shared_ptr old_data;
      frontend->frame_queue_.pop(old_data);
      frontend->frame_queue_.push(data);
    }

    auto next_time = prev_time 
        + std::chrono::duration<double>( data_reader->getTimestamp(frame_id+1) - data->timestamp_ );

    // std::cout << frame_id << " " << data_reader->getTimestamp(frame_id) << " " << data_reader->getTimestamp(frame_id+1) << std::endl;
    std::this_thread::sleep_until(next_time);
  }

  // Indicate the end of the sequence
  frontend->frame_queue_.push(nullptr);
  std::cout << "Finished frame data thread" << std::endl;
}

void receiveOutput() {
  std::cout << "Starting backend receiver thread" << std::endl;

  BackendOutput::shared_ptr data;
  while (true) {
    backend_vis_queue.pop(data);
    if (data.get()) {
      
      extrinsics = data->extrinsics_;

      segment_poses = data->segment_poses_;

      tracked_cloud = data->tracked_landmarks_;
      stereo_cloud = data->stereo_landmarks_;

      // Logging
      std::cout << std::fixed;
      std::cout << std::setprecision(5);
      double rad_to_deg = 180.0/3.14159265358979311600;
      std::vector<float> vals;
      vals.push_back(current_frame_id);
      for (int l = 0; l < data_reader->getNumCam(); l++) {
        gtsam::Pose3 T_diff = extrinsics_gt[l].inverse() * extrinsics[l];
        double t_err = T_diff.translation().norm();
        std::pair<gtsam::Unit3, double> ax_ang = T_diff.rotation().axisAngle();
        vals.push_back(t_err);
        vals.push_back(ax_ang.second);
        // std::cout << "Absolute Cam " << l << " errors:    t " << t_err << "     R " << ax_ang.second*rad_to_deg << std::endl;
      }
      for (auto p : stereo_pairs) {
        gtsam::Pose3 T_rel_gt = extrinsics_gt[p.first].inverse() * extrinsics_gt[p.second];
        gtsam::Pose3 T_rel_est = extrinsics[p.first].inverse() * extrinsics[p.second];
        gtsam::Pose3 T_diff = T_rel_gt.inverse() * T_rel_est;
        double t_err = T_diff.translation().norm();
        std::pair<gtsam::Unit3, double> ax_ang = T_diff.rotation().axisAngle();
        // std::cout << "Relative cam pair " << p.first << " " << p.second << " errors:    t " << t_err << "     R " << ax_ang.second*rad_to_deg << std::endl;
      }

      error_data_log.Log(vals);
    }
    else {
      break;
    }
  }

  std::cout << "Finished backend receiver thread" << std::endl;
}


int main(int argc, char** argv) {

  if (argc < 2) {
    printf("Need to specify dataset type as arg #1.\n");
    return 1;
  }
  else if (argc < 3) {
    printf("Need to specify dataset path as arg #2.\n");
    return 1;
  }

  data_reader = getDataReader(argv[1], argv[2]);
  if (!data_reader->initialize()) {
    printf("Dataset path not valid.\n");
    return 1;
  }
  const uint8_t NUM_CAM = data_reader->getNumCam();

  // Parameter loading
  std::string frontend_yaml = "../params/frontend.yaml";
  FrontendParams frontend_params(frontend_yaml);
  std::string backend_yaml = "../params/backend.yaml";
  BackendParams backend_params(backend_yaml);
  std::string stereo_yaml = "../params/stereo/" + std::string(argv[1]) + ".yaml";
  stereo::Params stereo_params = stereo::readParams(stereo_yaml);
  // Backend parameters dependent on dataset
  backend_params.setNumPartitions(2*data_reader->getNumCam());
  std::string database_dir = std::string("../database/") + argv[1] + "/";
  backend_params.setDatabaseDirectory(database_dir);
  if (argc >= 5)
    backend_params.seg_database_.max_segments_ = std::stoi(argv[4]);

  // Setup simulated odometry drift
  std::string sim_yaml = "../params/sim.yaml";
  if (argc >= 4)
    sim_yaml = argv[3];
  SimParams sim_params(sim_yaml);
  odometry_sampler = std::make_unique<gtsam::Sampler<gtsam::Pose3>>
      (sim_params.odom_noise_model_, sim_params.odom_seed_);
  T_wv_noisy_prev = gtsam::Pose3::identity();

  // Frontend and backend constructors
  frontend = std::make_unique<Frontend>(frontend_params);
  segment_construction = std::make_unique<SegmentConstruction>(backend_params, stereo_params);
  database_thread = std::make_unique<DatabaseThread>(backend_params);

  // Camera loading
  std::string extrinsics_path = database_dir + std::string("extrinsics.dat");
  // Intrinsics
  std::vector<CameraModelType> camera_types = data_reader->getCameraModelType();
  std::vector<gtsam::Vector> camera_params = data_reader->getCalibParams();
  for (int l=0; l<NUM_CAM; l++) {
    intrinsics.push_back( CameraModelFactory::getCameraModel(
        camera_types[l], camera_params[l], frontend_params.getUndistortParams() ) );
  }
  // Extrinsics
  extrinsics_gt = data_reader->getExtrinsics();
  // Stereo pairs
  stereo_pairs = stereo_params.stereo_pairs_;

  // Load extrinsics if available, otherwise create noisy estimates for now
  std::vector<gtsam::Pose3> extrinsics_init;
  gtsam::Sampler<gtsam::Pose3> extrinsics_sampler(sim_params.extrinsics_noise_model_, sim_params.extrinsics_seed_);
  // Introduce noise to other poses
  for (int l = 0; l < NUM_CAM; l++) {
    gtsam::Pose3 sample = extrinsics_sampler.sample(extrinsics_gt[l]);
    extrinsics_init.push_back(sample);
  }
  if (database_thread->loadDatabase(extrinsics_init)) {
    std::cout << "Loaded extrinsics from database" << std::endl;
  }
  else {
    std::cout << "Crerated noisy initialization of extrinsics" << std::endl;
  }
  extrinsics = extrinsics_init;

  // Log initial errors
  std::vector<float> vals;
  vals.push_back(0 /*frame_id*/);
  for (int l = 0; l < NUM_CAM; l++) {
    gtsam::Pose3 T_diff = extrinsics_gt[l].inverse() * extrinsics[l];
    double t_err = T_diff.translation().norm();
    std::pair<gtsam::Unit3, double> ax_ang = T_diff.rotation().axisAngle();
    vals.push_back(t_err);
    vals.push_back(ax_ang.second);
    std::cout << "Init error: " << l << " " << t_err << " " << ax_ang.second << std::endl;
  }
  error_data_log.Log(vals);


  // Connect queues
  frontend->backend_queue_ = &segment_construction->input_queue_;
  frontend->vis_queue_ = &frontend_vis_queue;
  segment_construction->output_queue_ = &database_thread->input_queue_;
  database_thread->output_queue_ = &backend_vis_queue;
  // TODO: Tune queue sizes for real-time operation
  frontend->frame_queue_.set_capacity(10);
  segment_construction->input_queue_.set_capacity(10);
  database_thread->input_queue_.set_capacity(3);
  frontend_vis_queue.set_capacity(10);
  backend_vis_queue.set_capacity(5);

  // Initialize pangolin
  auto& window = pangolin::CreateWindowAndBind("Main", 1920/2, 1080/2, 
      pangolin::Params({{"HIGHRES", "true"}}) );
  glEnable(GL_DEPTH_TEST);
  pangolin::View& main_display = pangolin::CreateDisplay().SetBounds(
        0.2, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0);
  pangolin::View& img_view_display =
      pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, 
              0.0,
              /*pangolin::Attach::Pix(IMG_VIEW_WIDTH))*/ 0.5)
          .SetLayout(pangolin::LayoutEqual);


  gtsam::Point3 cam_back(0, -10, -6);
  gtsam::Point3 cam_front(0, 0, 2);
  // gtsam::Point3 cam_back(0, 1, -2);
  // gtsam::Point3 cam_front(0, 0, 2);
  // gtsam::Point3 cam_back(-10, -0.0, 5.0);
  // gtsam::Point3 cam_front(10.0, 0.0, -5.0);
  gtsam::Pose3 pose_start = data_reader->getGroundTruthOdom(0) * extrinsics_gt[data_reader->getBaseCam()];
  cam_back = pose_start * cam_back;
  cam_front = pose_start * cam_front;
  camera = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(640, 480, 400, 400, 320, 240, 0.001, 10000),
      pangolin::ModelViewLookAt(cam_back(0), cam_back(1), cam_back(2), 
                                cam_front(0), cam_front(1), cam_front(2),
                                pangolin::AxisZ));
  pangolin::View& display3D =
        pangolin::CreateDisplay()
            .SetAspect(-640 / 480.0)
            .SetBounds(0.0, 1.0, /*pangolin::Attach::Pix(IMG_VIEW_WIDTH)*/ 0.5, 1.0)
            .SetHandler(new pangolin::Handler3D(camera));
  display3D.extern_draw_function = drawScene;

  pangolin::View& plot_display = pangolin::CreateDisplay().SetBounds(
      0.0, 0.2, pangolin::Attach::Pix(UI_WIDTH), 1.0);

  plotter = new pangolin::Plotter(&error_data_log, 
      -70.0, data_reader->getNumFrames(), 
      -0.0025, 0.10, 20.0f, 0.01f);
  plot_display.AddDisplay(*plotter);

  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
      pangolin::Attach::Pix(UI_WIDTH));
      
  std::vector<std::shared_ptr<pangolin::ImageView> > img_view;
  while (img_view.size() < NUM_CAM) {
    std::shared_ptr<pangolin::ImageView> iv(new pangolin::ImageView);

    size_t idx = img_view.size();
    img_view.push_back(iv);

    img_view_display.AddDisplay(*iv);
    iv->extern_draw_function = std::bind(&drawFeatures, std::placeholders::_1, idx);;
  }
  main_display.AddDisplay(display3D);
  main_display.AddDisplay(img_view_display);

  // Resizing removes issue with render shown in bottom 1/4
  window.Resize(1920/2 + 16, 1080/2 + 9);

  // Start processing threads
  frontend->initialize();
  segment_construction->initialize();
  database_thread->initialize();

  // Start IO and visualization threads
  std::thread t1(&feedFrames);
  std::thread t2(&receiveOutput);

  // pangolin::DisplayBase().RecordOnRender("ffmpeg:[fps=30,flip=true]//screencap.mp4");


  std::vector<cv::Mat> prev_imgs;
  while (!pangolin::ShouldQuit()) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    display3D.Activate(camera);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    img_view_display.Activate();

    // Frame visualization
    bool new_viz_frame = false;
    if (prev_imgs.size() == 0) {
      frontend_vis_queue.pop(multi_frame);
      new_viz_frame = true;
    }
    else {
      new_viz_frame = frontend_vis_queue.try_pop(multi_frame);
    }
    if (new_viz_frame) {
      if (multi_frame.get()) {

        std::vector<cv::Mat> imgs;
        // Approx world pose
        T_wv_est = multi_frame->T_wv_est_;
        const gtsam::Pose3& T_vc0 = extrinsics[0];
        gtsam::Pose3 viz_pose = gtsam::Pose3(gtsam::Rot3::identity(), (T_wv_est*T_vc0).translation());
        // camera.Follow(viz_pose.matrix());
        // gtsam::Point3 cam_back(-8.0*std::sin(multi_frame->frame_id_/50.0), -8.0*std::cos(multi_frame->frame_id_/50.0), 5.0);
        // gtsam::Point3 cam_front(8.0*std::sin(multi_frame->frame_id_/50.0), 8.0*std::cos(multi_frame->frame_id_/50.0), -5.0);
        // camera = pangolin::OpenGlRenderState(
        //     pangolin::ProjectionMatrix(640, 480, 400, 400, 320, 240, 0.001, 10000),
        //     pangolin::ModelViewLookAt(cam_back(0), cam_back(1), cam_back(2), 
        //                               cam_front(0), cam_front(1), cam_front(2),
        //                               pangolin::AxisZ));
        // Show images
        for (size_t cam_id = 0; cam_id < NUM_CAM; cam_id++) {
          cv::Mat img;
          if (!show_mask) {
            img = multi_frame->frames_[cam_id]->img_;
          }
          else {
            img = multi_frame->masks_[cam_id];
            cv::bitwise_not(img, img);
          }

          pangolin::GlPixFormat fmt;
          fmt.glformat = GL_LUMINANCE;
          fmt.gltype = GL_UNSIGNED_BYTE;
          fmt.scalable_internal_format = GL_LUMINANCE8;

          img_view[cam_id]->SetImage(img.data, img.cols, img.rows, img.step, fmt);

          imgs.push_back(img);
        }

        prev_imgs = imgs;
      }
      else {
        break;
      }
    }
    else {
      for (size_t cam_id = 0; cam_id < NUM_CAM; cam_id++) {
        cv::Mat img = prev_imgs[cam_id];

        pangolin::GlPixFormat fmt;
        fmt.glformat = GL_LUMINANCE;
        fmt.gltype = GL_UNSIGNED_BYTE;
        fmt.scalable_internal_format = GL_LUMINANCE8;

        img_view[cam_id]->SetImage(img.data, img.cols, img.rows, img.step, fmt);
      }
    }

    // TODO: Only call this if backend output received?
    drawPlots();

    pangolin::FinishFrame();
  }

  t2.join();
  t1.join();
  frontend->join();
  segment_construction->join();
  database_thread->join();

  std::cout << "Serializing extrinsics estimates to " << extrinsics_path << std::endl;
  io::serializeExtrinsics(extrinsics, extrinsics_path);


  std::ofstream output_file;
  output_file.open("../calibration/logs/" + std::string(argv[1]) + ".txt", std::ofstream::out | std::ofstream::app);
  output_file << argv[2] << "," << sim_yaml << "," << backend_params.seg_database_.max_segments_ << "," << static_cast<int>(NUM_CAM) << "," << stereo_pairs.size();

  // Log final errors
  std::vector<double> errors;
  for (int l = 0; l < NUM_CAM; l++) {
    gtsam::Pose3 T_diff = extrinsics_gt[l].inverse() * extrinsics[l];
    double t_err = T_diff.translation().norm();
    std::pair<gtsam::Unit3, double> ax_ang = T_diff.rotation().axisAngle();
    std::cout << "Final absolute error: " << l << " " << t_err << " " << ax_ang.second << std::endl;
    std::cout << T_diff << std::endl;
    output_file << "," << t_err << "," << ax_ang.second;
  }
  for (auto p : stereo_pairs) {
    gtsam::Pose3 T_rel_gt = extrinsics_gt[p.first].inverse() * extrinsics_gt[p.second];
    gtsam::Pose3 T_rel_est = extrinsics[p.first].inverse() * extrinsics[p.second];
    gtsam::Pose3 T_diff = T_rel_gt.inverse() * T_rel_est;
    double t_err = T_diff.translation().norm();
    std::pair<gtsam::Unit3, double> ax_ang = T_diff.rotation().axisAngle();
    double rel_dot = T_rel_gt.translation().normalized().dot(T_rel_est.translation().normalized());
    rel_dot = rel_dot > 1.0 ? 1.0 : (rel_dot < -1.0 ? -1.0 : rel_dot);
    double baseline_ang_error = std::acos(rel_dot);
    std::cout << "Final relative cam pair error: " << p.first << " " << p.second << " " << t_err << " " << ax_ang.second << " " << baseline_ang_error << std::endl;
    output_file << "," << t_err << "," << ax_ang.second << "," << baseline_ang_error;
  }
  output_file << std::endl;
  output_file.close();

  std::cout << "Main thread ended" << std::endl;

  return 0;
}


// Pangolin function definitions

void drawScene(pangolin::View& view) {
  // UNUSED(view);
  view.Activate(camera);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

  glPointSize(5);
  glColor3f(1.0, 0.0, 0.0);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  if (show_est_extrinsics) {
    for (const auto& ext : extrinsics) {
      render_camera((T_wv_est*ext).matrix(), 2.0f, est_cam_color, 0.05f);
    }
  }
  if (show_gt_extrinsics) {
    for (const auto& ext : extrinsics_gt) {
      render_camera((T_wv_est*ext).matrix(), 2.0f, gt_cam_color, 0.05f);
    }
  }

  if (show_tracked_points) {
    glColor3ubv(tracked_point_color);
    pangolin::glDrawPoints(tracked_cloud);
  }
  if (show_stereo_points) {
    glColor3ubv(stereo_point_color);
    pangolin::glDrawPoints(stereo_cloud);
  }
  if (show_segment_poses) {
    for (int i = 0; i < segment_poses.size(); i++) {
      if (segment_poses[i].size() > 0) {
        gtsam::Pose3 T_wv_prev = segment_poses[i][0];
        for (const auto& ext : extrinsics)
          render_camera((T_wv_prev*ext).matrix(), 2.0f, segment_pose_color, 0.05f);
        for (const auto& ext : extrinsics_gt)
          render_camera((T_wv_prev*ext).matrix(), 2.0f, gt_cam_color, 0.05f);

        for (int j = 1; j < segment_poses[i].size(); j++) {
          gtsam::Pose3 T_wv = segment_poses[i][j];
          for (const auto& ext : extrinsics)
            render_camera((T_wv*ext).matrix(), 2.0f, segment_pose_color, 0.05f);
          for (const auto& ext : extrinsics_gt)
            render_camera((T_wv*ext).matrix(), 2.0f, gt_cam_color, 0.05f);

          const std::vector<Eigen::Vector3d> lines 
              = {T_wv_prev.translation(), T_wv.translation()};
          glColor3ubv(segment_pose_color);
          glLineWidth(1.0f);
          pangolin::glDrawLines(lines);

          T_wv_prev = T_wv;
        }
      }
    }
  }

  pangolin::glDrawAxis(gtsam::Pose3::identity().matrix(), 1.0);
}

void drawFeatures(pangolin::View& v, size_t cam_id) {
  if (show_features) {
    glLineWidth(1.0);
    glColor3f(1.0, 0.0, 0.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    const auto& frame = multi_frame->frames_[cam_id];

    size_t min_id = std::numeric_limits<size_t>::max();
    size_t max_id = 0;

    for (const auto& ft_id : frame->feature_ids_) {
      min_id = std::min(ft_id, min_id);
      max_id = std::max(ft_id, max_id);
    }

    for (size_t i = 0; i < frame->features_.size(); i++) {
      const float radius = 6.5;
      const auto& ft = frame->features_[i];
      const auto& id = frame->feature_ids_[i];

      float r, g, b;
      getcolor(id - min_id, max_id - min_id, b, g, r);
      glColor3f(r, g, b);

      pangolin::glDrawCirclePerimeter(ft.x, ft.y, radius);
    }

    // glColor3f(0.3, 0.0, 0.0);
    // pangolin::GlFont(pangolin::AnonymousPro_ttf, 25)
    //     .Text("%d fts", frame->features_.size())
    //     .Draw(20, 30);
  }
}

void drawPlots() {
  plotter->ClearSeries();
  plotter->ClearMarkers();

  for (int l = 0; l < data_reader->getNumCam(); l++) {
    int ind_t = 2*l + 1;
    int ind_R = 2*l + 2;
    std::string str_t = std::to_string(ind_t);
    std::string str_R = std::to_string(ind_R);
    std::string label_R("");
    std::string label_t("");
    if (l == 0) {
      label_R = "Rotation error (radians)";
      label_t = "Translation error (meters)";
    }
    plotter->AddSeries("$0", "$" + str_t, pangolin::DrawingModeLine,
                        pangolin::Colour::Red(), label_t, &error_data_log);
    plotter->AddSeries("$0", "$" + str_R, pangolin::DrawingModeLine,
                        pangolin::Colour::Green(), label_R, &error_data_log);
  }
}

void render_camera(const Eigen::Matrix4d& T_w_c, double lineWidth,
                    const u_int8_t* color, double sizeFactor) {
  const double sz = sizeFactor;
  const double width = 640, height = 480, fx = 500, fy = 500, cx = 320, cy = 240;

  const std::vector<Eigen::Vector3d> lines = {
      {0, 0, 0},
      {sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz},
      {0, 0, 0},
      {sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
      {0, 0, 0},
      {sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
      {0, 0, 0},
      {sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz},
      {sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz},
      {sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
      {sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
      {sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
      {sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
      {sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz},
      {sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz},
      {sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz}};

  glPushMatrix();
  glMultMatrixd(T_w_c.data());
  glColor3ubv(color);
  glLineWidth(lineWidth);
  pangolin::glDrawLines(lines);
  glPopMatrix();
}

void getcolor(float p, float np, float& r, float& g, float& b) {
  float inc = 4.0 / np;
  float x = p * inc;
  r = 0.0f;
  g = 0.0f;
  b = 0.0f;

  if ((0 <= x && x <= 1) || (5 <= x && x <= 6))
    r = 1.0f;
  else if (4 <= x && x <= 5)
    r = x - 4;
  else if (1 <= x && x <= 2)
    r = 1.0f - (x - 1);

  if (1 <= x && x <= 3)
    g = 1.0f;
  else if (0 <= x && x <= 1)
    g = x - 0;
  else if (3 <= x && x <= 4)
    g = 1.0f - (x - 3);

  if (3 <= x && x <= 5)
    b = 1.0f;
  else if (2 <= x && x <= 3)
    b = x - 2;
  else if (5 <= x && x <= 6)
    b = 1.0f - (x - 5);
}