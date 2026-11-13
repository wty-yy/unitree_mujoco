#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include <mujoco/mujoco.h>

// Yaw-aligned grid: x[-0.8,0.8] + offset_x, y[-0.5,0.5], x outer/y inner.
inline std::vector<float> sampleHeightMap(const mjModel* model, const mjData* data,
                                         std::vector<mjvGeom>* visualization_geoms = nullptr,
                                         double offset_x = 0.0)
{
    if (visualization_geoms) {
        visualization_geoms->clear();
        visualization_geoms->reserve(187);
    }
    const int pelvis = mj_name2id(model, mjOBJ_BODY, "pelvis");
    if (pelvis < 0) throw std::runtime_error("Height map requires pelvis body");
    const mjtNum* base = data->xpos + 3 * pelvis;
    const mjtNum* q = data->xquat + 4 * pelvis;
    const double yaw = std::atan2(2 * (q[0]*q[3] + q[1]*q[2]),
                                  1 - 2 * (q[2]*q[2] + q[3]*q[3]));
    const double c = std::cos(yaw), s = std::sin(yaw);
    const mjtNum direction[3] = {0, 0, -1};
    std::vector<float> heights(187, std::numeric_limits<float>::quiet_NaN());
    // Only world-welded collision geometry is terrain; exclude all robot links.
    std::vector<int> terrain;
    for (int g = 0; g < model->ngeom; ++g) {
        if (model->body_weldid[model->geom_bodyid[g]] == 0 &&
            (model->geom_contype[g] || model->geom_conaffinity[g])) terrain.push_back(g);
    }
    for (int x = 0; x < 17; ++x) {
        for (int y = 0; y < 11; ++y) {
            const double dx = -0.8 + offset_x + 0.1*x, dy = -0.5 + 0.1*y;
            const mjtNum start[3] = {base[0] + c*dx - s*dy,
                                     base[1] + s*dx + c*dy, base[2] + 20};
            double closest = 100.0;
            bool hit = false;
            for (int g : terrain) {
                const int type = model->geom_type[g];
                const double distance = type == mjGEOM_MESH
                    ? mj_rayMesh(model, data, g, start, direction, nullptr)
                    : type == mjGEOM_HFIELD
                    ? mj_rayHfield(model, data, g, start, direction, nullptr)
                    : mju_rayGeom(data->geom_xpos + 3*g, data->geom_xmat + 9*g,
                                  model->geom_size + 3*g, start, direction, type, nullptr);
                if (distance >= 0 && distance < closest) { closest = distance; hit = true; }
            }
            if (hit) {
                heights[x*11+y] = std::clamp(20.0 - closest, -2.0, 2.0);
                if (visualization_geoms) {
                    const mjtNum radius = 0.012;
                    const mjtNum size[3] = {radius, radius, radius};
                    // Draw the actual terrain hit, independent of policy clipping.
                    const mjtNum position[3] = {start[0], start[1], start[2] - closest + radius};
                    const float color[4] = {0.1f, 1.0f, 0.3f, 1.0f};
                    mjvGeom point;
                    mjv_initGeom(&point, mjGEOM_SPHERE, size, position, nullptr, color);
                    point.category = mjCAT_DECOR;
                    point.objtype = mjOBJ_UNKNOWN;
                    point.objid = -1;
                    visualization_geoms->push_back(point);
                }
            }
        }
    }
    return heights;
}
