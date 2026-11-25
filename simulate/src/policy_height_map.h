#pragma once

#include <mujoco/mujoco.h>
#include <unitree/idl/go2/HeightMap_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include "camera_height_map.h"

class PolicyHeightMapPublisher {
    using Message = unitree_go::msg::dds_::HeightMap_;
public:
    explicit PolicyHeightMapPublisher(const mjModel* model, bool forward_elevation = false, bool shift_elevation_forward = true, float forward_offset = 0.3f,
                                     const CameraHeightMapConfig& camera_config = {})
        : ray_model_(mj_copyModel(nullptr, model), mj_deleteModel),
          elevation_("rt/go2/policy_height_map"), missing_mask_("rt/go2/policy_height_map_missing_mask"),
          width_(forward_elevation ? 9 : 17), origin_x_((forward_elevation ? 0.0f : -0.8f) + (shift_elevation_forward ? forward_offset : 0.0f))
    {
        for (int i = 0; i < model->ngeom; ++i) {
            ray_model_->geom_group[i] = isHeightMapTerrain(model, i) ? 0 : 1;
        }
        elevation_.InitChannel();
        missing_mask_.InitChannel();
        visualization_geoms_.reserve(width_ * 11);
        if (camera_config.enabled) camera_ = std::make_unique<CameraHeightMap>(model, width_, origin_x_, camera_config);
    }

    const std::vector<mjvGeom>& visualization_geoms() const { return visualization_geoms_; }

    std::string overlay_text() const {
        if (!camera_ || !camera_->config().debug) return "";
        return "Camera height map: " + std::to_string(int(camera_->config().camera_fps)) + " Hz, delay " +
            std::to_string(camera_->delay()) + " frames" + (camera_->full_view() ? " (training full-view episode)" : "") +
            "\nGray: unknown | Green: measured | Yellow: candidates\nBlue: policy input | Red: input bounds | Orange: camera";
    }

    bool update(const mjData* data)
    {
        if (data->time >= last_sim_time_ && data->time - last_sim_time_ < 0.02 - 1e-8) return false;
        if (data->time < last_sim_time_) last_debug_frame_ = -1;
        last_sim_time_ = data->time;
        if (camera_) return update_camera(data);
        visualization_geoms_.clear();
        Message height, missing;
        const double stamp = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        for (auto* message : {&height, &missing}) {
            message->stamp(stamp);
            message->frame_id("base_yaw_relative");
            message->resolution(0.1f);
            message->width(width_);
            message->height(11);
            message->origin({origin_x_, -0.5f});
            // Simulation exposes the complete terrain, without sensor occlusion.
            message->data().resize(width_ * 11, 0.0f);
        }
        const auto* q = data->qpos + 3;
        const double yaw = std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
        const mjtNum direction[3] = {0, 0, -1};
        const mjtByte groups[6] = {1, 0, 0, 0, 0, 0};
        for (int y = 0; y < 11; ++y) for (int x = 0; x < width_; ++x) {
            const double lx = origin_x_ + 0.1 * x, ly = -0.5 + 0.1 * y;
            const mjtNum start[3] = {data->qpos[0] + std::cos(yaw) * lx - std::sin(yaw) * ly,
                data->qpos[1] + std::sin(yaw) * lx + std::cos(yaw) * ly, data->qpos[2] + 20.0};
            int geom = -1;
            const double distance = mj_ray(ray_model_.get(), data, start, direction, groups, 1, -1, &geom, nullptr);
            if (distance >= 0 && distance <= 100) {
                height.data()[y * width_ + x] = std::clamp(20.0 - distance, -2.0, 2.0);
                // Keep the actual hit height, independent of policy clipping.
                // Lift the centre by the radius so the dot rests on the surface.
                const mjtNum radius = 0.012;
                const mjtNum size[3] = {radius, radius, radius};
                const mjtNum position[3] = {start[0], start[1], start[2] - distance + radius};
                const float color[4] = {0.1f, 1.0f, 0.3f, 1.0f};
                mjvGeom point;
                mjv_initGeom(&point, mjGEOM_SPHERE, size, position, nullptr, color);
                point.category = mjCAT_DECOR;
                point.objtype = mjOBJ_UNKNOWN;
                point.objid = -1;
                visualization_geoms_.push_back(point);
            } else {
                missing.data()[y * width_ + x] = 1.0f;
            }
        }
        elevation_.Write(height);
        missing_mask_.Write(missing);
        return true;
    }
private:
    bool update_camera(const mjData* data) {
        const bool acquired = camera_->update(data);
        if (acquired) {
            Message height, missing;
            const double stamp = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
            for (auto* message : {&height, &missing}) {
                message->stamp(stamp); message->frame_id("base_yaw_relative"); message->resolution(.1f);
                message->width(width_); message->height(11); message->origin({origin_x_, -.5f});
                message->data().resize(width_ * 11);
            }
            const auto& image = camera_->output();
            for (int x=0;x<width_;++x) for (int y=0;y<11;++y) {
                height.data()[y*width_+x]=image.elevation[x*11+y];
                missing.data()[y*width_+x]=image.missing[x*11+y];
            }
            elevation_.Write(height); missing_mask_.Write(missing);
        }
        const long debug_frame = static_cast<long>(std::floor(data->time / (.02 * camera_->config().debug_interval) + 1e-8));
        if (camera_->config().debug && debug_frame == last_debug_frame_) return acquired;
        if (!camera_->config().debug && !acquired) return false;
        last_debug_frame_ = debug_frame;
        visualization_geoms_.clear();
        const auto& image = camera_->output();
        std::vector<CameraHeightMap::DebugPoint> points;
        std::vector<CameraHeightMap::DebugLine> lines;
        if (camera_->config().debug) camera_->debug(data, points, lines);
        else {
            const auto* q=data->qpos+3;
            const double yaw=std::atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
            for(int x=0;x<width_;++x) for(int y=0;y<11;++y) if(image.missing[x*11+y]==0) {
                const double lx=origin_x_+.1*x, ly=-.5+.1*y;
                points.push_back({{data->qpos[0]+cos(yaw)*lx-sin(yaw)*ly,
                                   data->qpos[1]+sin(yaw)*lx+cos(yaw)*ly,
                                   data->qpos[2]+image.elevation[x*11+y]},3});
            }
        }
        const float colors[4][4]={{55/255.f,62/255.f,72/255.f,1},{0,155/255.f,45/255.f,1},
                                  {1,210/255.f,0,1},{0,65/255.f,245/255.f,1}};
        const double radii[4]={.014,.023,.023,.033};
        for (const auto& p:points) {
            const mjtNum size[3]={radii[p.layer],radii[p.layer],radii[p.layer]};
            mjvGeom geom; mjv_initGeom(&geom,mjGEOM_SPHERE,size,p.position.data(),nullptr,colors[p.layer]);
            geom.category=mjCAT_DECOR; geom.objtype=mjOBJ_UNKNOWN; geom.objid=-1;
            visualization_geoms_.push_back(geom);
        }
        const float line_colors[2][4]={{1,65/255.f,70/255.f,1},{1,140/255.f,0,1}};
        for (const auto& l:lines) {
            mjvGeom geom; mjv_initGeom(&geom,mjGEOM_CYLINDER,nullptr,nullptr,nullptr,line_colors[l.layer]);
            mjv_connector(&geom,mjGEOM_CYLINDER,l.layer==0?.0075:.004,l.from.data(),l.to.data());
            geom.category=mjCAT_DECOR; geom.objtype=mjOBJ_UNKNOWN; geom.objid=-1;
            visualization_geoms_.push_back(geom);
        }
        return true;
    }

    std::unique_ptr<CameraHeightMap> camera_;
    long last_debug_frame_ = -1;
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> ray_model_;
    unitree::robot::ChannelPublisher<Message> elevation_, missing_mask_;
    double last_sim_time_ = -1;
    std::vector<mjvGeom> visualization_geoms_;
    int width_;
    float origin_x_;
};
