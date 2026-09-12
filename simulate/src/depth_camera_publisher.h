#pragma once

#include <atomic>
#include <thread>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

namespace mujoco
{
class Simulate;
}

class DepthCameraPublisher
{
public:
    DepthCameraPublisher(const mjModel* model, const mjData* data,
                         mujoco::Simulate* simulate, GLFWwindow* render_window);
    ~DepthCameraPublisher();

    DepthCameraPublisher(const DepthCameraPublisher&) = delete;
    DepthCameraPublisher& operator=(const DepthCameraPublisher&) = delete;

    void start();

private:
    void run();

    const mjModel* model_;
    const mjData* data_;
    mujoco::Simulate* simulate_;
    GLFWwindow* render_window_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
};
