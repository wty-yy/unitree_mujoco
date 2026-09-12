#include "depth_camera_publisher.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include "dds/DepthObservation_.hpp"
#include "param.h"
#include "simulate.h"

namespace
{
using DepthObservation = unitree_sim::msg::dds_::DepthObservation_;

constexpr char kDepthPreviewWindowName[] = "D435i raw depth (grayscale)";
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

bool ValidateCameraProjection(const mjModel* model, int camera_id, int width, int height)
{
    const int* resolution = model->cam_resolution + 2 * camera_id;
    if (resolution[0] != width || resolution[1] != height)
    {
        std::cerr << "Depth camera viewport " << width << "x" << height
                  << " does not match model resolution " << resolution[0]
                  << "x" << resolution[1] << std::endl;
        return false;
    }

    const float* intrinsic = model->cam_intrinsic + 4 * camera_id;
    const float* sensor_size = model->cam_sensorsize + 2 * camera_id;
    if (intrinsic[0] <= 0.0F || intrinsic[1] <= 0.0F ||
        sensor_size[0] <= 0.0F || sensor_size[1] <= 0.0F)
    {
        std::cerr << "Depth camera requires positive focal lengths and sensor size"
                  << std::endl;
        return false;
    }
    if (std::abs(intrinsic[2]) > 1.0e-5F || std::abs(intrinsic[3]) > 1.0e-5F)
    {
        std::cerr << "Depth camera principal-point offset must be centered; got ("
                  << intrinsic[2] << ", " << intrinsic[3] << ")" << std::endl;
        return false;
    }

    const double authored_horizontal_fov = 2.0 * std::atan(
        sensor_size[0] / (2.0 * intrinsic[0])) * kRadiansToDegrees;
    const double authored_vertical_fov = 2.0 * std::atan(
        sensor_size[1] / (2.0 * intrinsic[1])) * kRadiansToDegrees;
    const double rendered_vertical_fov = model->cam_fovy[camera_id];
    const double rendered_horizontal_fov = 2.0 * std::atan(
        (static_cast<double>(width) / height) *
        std::tan(rendered_vertical_fov / (2.0 * kRadiansToDegrees))) *
        kRadiansToDegrees;

    // MuJoCo's classic renderer derives horizontal FOV from vertical FOV and
    // viewport aspect ratio, while focalpixel can encode a small pixel-aspect
    // difference. The reference calibration differs by only about 0.17 deg.
    constexpr double projection_tolerance_degrees = 0.5;
    if (std::abs(rendered_horizontal_fov - authored_horizontal_fov) >
            projection_tolerance_degrees ||
        std::abs(rendered_vertical_fov - authored_vertical_fov) >
            projection_tolerance_degrees)
    {
        std::cerr << "Depth camera rendered FOV (" << rendered_horizontal_fov
                  << ", " << rendered_vertical_fov
                  << ") deg does not match authored FOV ("
                  << authored_horizontal_fov << ", " << authored_vertical_fov
                  << ") deg" << std::endl;
        return false;
    }

    std::cout << "Depth camera projection: viewport=" << width << "x" << height
              << ", rendered FOV=" << rendered_horizontal_fov << "x"
              << rendered_vertical_fov << " deg, authored FOV="
              << authored_horizontal_fov << "x" << authored_vertical_fov
              << " deg, principal offset=(" << intrinsic[2] << ", "
              << intrinsic[3] << ") px" << std::endl;
    return true;
}

bool IsRobotGeom(const mjModel* model, int geom_id, int robot_root_body_id)
{
    if (geom_id < 0 || geom_id >= model->ngeom)
    {
        return false;
    }

    int body_id = model->geom_bodyid[geom_id];
    while (body_id > 0 && body_id != robot_root_body_id)
    {
        body_id = model->body_parentid[body_id];
    }
    return body_id == robot_root_body_id;
}

void RemoveRobotGeometry(const mjModel* model, int robot_root_body_id, mjvScene& scene)
{
    int output_index = 0;
    for (int input_index = 0; input_index < scene.ngeom; ++input_index)
    {
        const mjvGeom& geom = scene.geoms[input_index];
        if (geom.objtype == mjOBJ_GEOM && IsRobotGeom(model, geom.objid, robot_root_body_id))
        {
            continue;
        }
        if (output_index != input_index)
        {
            scene.geoms[output_index] = geom;
        }
        ++output_index;
    }
    scene.ngeom = output_index;
}

void ConvertDepthToMeters(const mjrContext& context, float near_plane, float far_plane,
                          int width, int height, const std::vector<float>& depth_buffer,
                          std::vector<float>& depth_meters)
{
    const float range = far_plane - near_plane;
    const float numerator = near_plane * far_plane;

    // OpenGL returns rows bottom-up. DDS uses conventional top-left image order.
    for (int y = 0; y < height; ++y)
    {
        const float* source = depth_buffer.data() + (height - 1 - y) * width;
        float* destination = depth_meters.data() + y * width;
        for (int x = 0; x < width; ++x)
        {
            const float z = source[x];
            if (context.readDepthMap == mjDEPTH_ZEROFAR)
            {
                destination[x] = numerator / (near_plane + z * range);
            }
            else
            {
                destination[x] = numerator / (far_plane - z * range);
            }
        }
    }
}

void ShowDepthPreview(const std::vector<float>& depth, int width, int height)
{
    const cv::Mat depth_view(height, width, CV_32FC1, const_cast<float*>(depth.data()));
    cv::Mat clipped;
    cv::Mat grayscale;
    cv::max(depth_view, 0.0, clipped);
    cv::min(clipped, param::config.depth_display_max, clipped);
    clipped.convertTo(grayscale, CV_8UC1, 255.0 / param::config.depth_display_max);
    cv::imshow(kDepthPreviewWindowName, grayscale);
    cv::waitKey(1);
}
}  // namespace

DepthCameraPublisher::DepthCameraPublisher(const mjModel* model, const mjData* data,
                                           mujoco::Simulate* simulate, GLFWwindow* render_window)
    : model_(model), data_(data), simulate_(simulate), render_window_(render_window)
{
    if (!model_ || !data_ || !simulate_ || !render_window_)
    {
        throw std::invalid_argument("DepthCameraPublisher received a null dependency");
    }
}

DepthCameraPublisher::~DepthCameraPublisher()
{
    stop_.store(true);
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void DepthCameraPublisher::start()
{
    if (!thread_.joinable())
    {
        thread_ = std::thread(&DepthCameraPublisher::run, this);
    }
}

void DepthCameraPublisher::run()
{
    const int camera_id = mj_name2id(model_, mjOBJ_CAMERA, param::config.camera_name.c_str());
    if (camera_id < 0)
    {
        std::cerr << "Depth camera not found: " << param::config.camera_name << std::endl;
        return;
    }
    if (!ValidateCameraProjection(
            model_, camera_id, param::config.camera_width, param::config.camera_height))
    {
        return;
    }
    const int robot_root_body_id = mj_name2id(model_, mjOBJ_BODY, "base_link");
    if (robot_root_body_id < 0)
    {
        std::cerr << "Depth camera rendering requires the Go2 base_link body" << std::endl;
        return;
    }

    unitree::robot::ChannelPublisher<DepthObservation> publisher(param::config.camera_topic);
    publisher.InitChannel();

    glfwMakeContextCurrent(render_window_);

    mjvCamera camera;
    mjv_defaultCamera(&camera);
    camera.type = mjCAMERA_FIXED;
    camera.fixedcamid = camera_id;

    mjvOption option;
    mjv_defaultOption(&option);

    mjvScene scene;
    mjv_defaultScene(&scene);
    mjv_makeScene(model_, &scene, 10000);

    mjrContext context;
    mjr_defaultContext(&context);
    mjr_makeContext(model_, &context, mjFONTSCALE_100);
    mjr_resizeOffscreen(param::config.camera_width, param::config.camera_height, &context);
    mjr_setBuffer(mjFB_OFFSCREEN, &context);

    mjData* render_data = mj_makeData(model_);
    if (!render_data)
    {
        std::cerr << "Could not allocate MuJoCo data for depth rendering" << std::endl;
        mjr_freeContext(&context);
        mjv_freeScene(&scene);
        glfwMakeContextCurrent(nullptr);
        return;
    }

    const int width = param::config.camera_width;
    const int height = param::config.camera_height;
    const mjrRect viewport{0, 0, width, height};
    const float near_plane = static_cast<float>(model_->vis.map.znear * model_->stat.extent);
    const float far_plane = static_cast<float>(model_->vis.map.zfar * model_->stat.extent);

    std::vector<float> depth_buffer(static_cast<std::size_t>(width) * height);
    std::vector<float> depth_meters(depth_buffer.size());

    DepthObservation message;
    message.frame_id(param::config.camera_name);
    message.width(static_cast<std::uint32_t>(width));
    message.height(static_cast<std::uint32_t>(height));
    message.encoding("32FC1");
    message.near_clip(near_plane);
    message.far_clip(far_plane);

    const double frame_period = 1.0 / param::config.camera_publish_hz;
    double next_frame_time = 0.0;
    double previous_sim_time = -1.0;
    std::uint64_t sequence = 0;

    std::cout << "Publishing raw D435i depth on " << param::config.camera_topic
              << " (" << width << "x" << height << " 32FC1, "
              << param::config.camera_publish_hz << " Hz)" << std::endl;

    while (!stop_.load() && !simulate_->exitrequest.load())
    {
        double sim_time = 0.0;
        {
            const std::unique_lock<std::recursive_mutex> lock(simulate_->mtx);
            sim_time = data_->time;
            if (sim_time + 1e-9 < previous_sim_time)
            {
                next_frame_time = sim_time;
            }
            if (sim_time + 1e-9 < next_frame_time)
            {
                // No new camera frame is due in simulation time.
                sim_time = -1.0;
            }
            else
            {
                mj_copyData(render_data, model_, data_);
            }
        }

        if (sim_time < 0.0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        previous_sim_time = sim_time;
        next_frame_time = sim_time + frame_period;

        mjv_updateScene(model_, render_data, &option, nullptr, &camera, mjCAT_ALL, &scene);
        // Training raycasts only against the environment. Filter the robot from
        // this private render scene without changing the model or UI scene.
        RemoveRobotGeometry(model_, robot_root_body_id, scene);
        mjr_render(viewport, &scene, &context);
        mjr_readPixels(nullptr, depth_buffer.data(), viewport, &context);
        ConvertDepthToMeters(context, near_plane, far_plane, width, height,
                             depth_buffer, depth_meters);

        message.frame_sequence(sequence++);
        message.sim_time(sim_time);
        message.data(depth_meters);
        if (!publisher.Write(message))
        {
            std::cerr << "Failed to publish depth frame " << message.frame_sequence() << std::endl;
        }

        if (param::config.show_depth)
        {
            ShowDepthPreview(depth_meters, width, height);
        }
    }

    if (param::config.show_depth)
    {
        cv::destroyWindow(kDepthPreviewWindowName);
    }
    publisher.CloseChannel();
    mj_deleteData(render_data);
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    glfwMakeContextCurrent(nullptr);
}
