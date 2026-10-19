#include "model_loader.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

using Model = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using Data = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

Model load(const std::filesystem::path& scene, const std::filesystem::path& model = {})
{
    char error[1024]{};
    Model loaded(model_loader::load(scene, model, error, sizeof(error)), mj_deleteModel);
    require(loaded != nullptr, scene.string() + ": " + error);
    return loaded;
}

void check_variant(const std::filesystem::path& path, bool shoes)
{
    const auto model = load(path);
    require(model->nq == 36 && model->nv == 35 && model->nu == 29,
            "Expected a floating base and 29 actuated joints");
    require(model->nexclude == 7, "Source SRDF exclusions were not preserved");
    int visuals = 0;
    std::array<int, mjNGEOMTYPES> collisions{};
    for (int i = 0; i < model->ngeom; ++i) {
        if (model->geom_contype[i] == 0 && model->geom_conaffinity[i] == 0) {
            ++visuals;
        } else {
            ++collisions[model->geom_type[i]];
        }
    }
    require(visuals == (shoes ? 46 : 34), "Source visual geometries were lost");
    require(collisions[mjGEOM_MESH] == 34, "Full mesh collisions were lost");
    require(collisions[mjGEOM_BOX] == (shoes ? 2 : 0) &&
            collisions[mjGEOM_CYLINDER] == (shoes ? 4 : 0),
            "Shoe sole collision shapes differ from the source URDF");
    require(model->nmesh == (shoes ? 40 : 34), "Source meshes were lost");

    const std::array<int, 3> sensor_types{
        mjSENS_JOINTPOS, mjSENS_JOINTVEL, mjSENS_JOINTACTFRC};
    for (int group = 0; group < 3; ++group) {
        for (int motor = 0; motor < model->nu; ++motor) {
            const int sensor = group * model->nu + motor;
            require(model->sensor_adr[sensor] == sensor &&
                    model->sensor_type[sensor] == sensor_types[group] &&
                    model->sensor_objid[sensor] == model->actuator_trnid[2 * motor],
                    "Motor and sensor ordering does not match the G1 DDS bridge");
        }
    }
    for (const char* name : {"imu_quat", "imu_gyro", "imu_acc", "frame_pos", "frame_vel",
                             "secondary_imu_quat", "secondary_imu_gyro", "secondary_imu_acc"}) {
        require(mj_name2id(model.get(), mjOBJ_SENSOR, name) >= 0,
                std::string("Missing bridge sensor: ") + name);
    }
}

void check_mode15_meshes(const mjModel* model, const mjModel* reference)
{
    require(model->nmesh == reference->nmesh, "Default scene must use the Mode 15 meshes");
    for (int mesh = 0; mesh < reference->nmesh; ++mesh) {
        const std::string name = mj_id2name(reference, mjOBJ_MESH, mesh);
        const int actual = mj_name2id(model, mjOBJ_MESH, name.c_str());
        require(actual >= 0, "Missing Mode 15 mesh: " + name);
        require(model->mesh_vertnum[actual] == reference->mesh_vertnum[mesh] &&
                model->mesh_facenum[actual] == reference->mesh_facenum[mesh],
                "Mode 15 mesh topology differs: " + name);
        const auto* vertices = reference->mesh_vert + 3 * reference->mesh_vertadr[mesh];
        const auto* faces = reference->mesh_face + 3 * reference->mesh_faceadr[mesh];
        require(std::equal(vertices, vertices + 3 * reference->mesh_vertnum[mesh],
                           model->mesh_vert + 3 * model->mesh_vertadr[actual]) &&
                std::equal(faces, faces + 3 * reference->mesh_facenum[mesh],
                           model->mesh_face + 3 * model->mesh_faceadr[actual]),
                "Mode 15 mesh geometry differs: " + name);
    }
}

int world_geoms(const mjModel* model)
{
    return std::count(model->geom_bodyid, model->geom_bodyid + model->ngeom, 0);
}

void check_physics(const mjModel* model)
{
    Data data(mj_makeData(model), mj_deleteData);
    mj_forward(model, data.get());
    for (int step = 0; step < 1000; ++step) {
        for (int motor = 0; motor < model->nu; ++motor) {
            const int joint = model->actuator_trnid[2 * motor];
            const double torque = -20 * data->qpos[model->jnt_qposadr[joint]] -
                                  2 * data->qvel[model->jnt_dofadr[joint]];
            data->ctrl[motor] = std::clamp(torque,
                model->actuator_ctrlrange[2 * motor], model->actuator_ctrlrange[2 * motor + 1]);
        }
        mj_step(model, data.get());
    }
    require(data->time > 1.9, "Simulation failed to advance");
    for (int i = 0; i < model->nq; ++i) {
        require(std::isfinite(data->qpos[i]), "Simulation produced non-finite positions");
    }
    for (int i = 0; i < mjNWARNING; ++i) {
        require(data->warning[i].number == 0, "Simulation reported a physics warning");
    }
}

int main(int argc, char** argv)
{
    try {
        require(argc == 2, "Pass the G1 model directory");
        const std::filesystem::path dir = std::filesystem::absolute(argv[1]);
        const auto complete = model_loader::resolve_model("complete", dir);
        const auto shoes = model_loader::resolve_model("complete-shoes", dir);
        require(model_loader::resolve_model("default", dir).empty(), "Default must preserve the scene model");
        require(model_loader::resolve_model(complete.filename(), dir) == complete,
                "Relative model XML paths do not resolve");
        require(model_loader::resolve_model(shoes, dir) == shoes,
                "Absolute model XML paths do not resolve");
        check_variant(complete, false);
        check_variant(shoes, true);

        const auto source_complete = load(complete);
        const auto source_shoes = load(shoes);
        for (const char* scene : {"flat.xml", "stairs.xml", "cross_stairs.xml", "cross_slope.xml",
                                  "race_track.xml", "scene_29dof.xml"}) {
            const auto baseline = load(dir / scene);
            check_mode15_meshes(baseline.get(), source_complete.get());
            if (std::string(scene) == "flat.xml") {
                check_physics(baseline.get());
            }
            for (const auto& path : {complete, shoes}) {
                const auto selected = load(dir / scene, path);
                const auto& source = path == complete ? source_complete : source_shoes;
                require(selected->ngeom - source->ngeom == baseline->ngeom - source_complete->ngeom,
                        std::string("Terrain geometry changed: ") + scene);
                require(world_geoms(selected.get()) == world_geoms(baseline.get()),
                        std::string("Terrain colliders changed: ") + scene);
                require(selected->nu == 29, "Actuators lost during scene composition");
                if (std::string(scene) == "flat.xml") {
                    check_physics(selected.get());
                }
            }
        }
        char error[1024]{};
        Model incompatible(model_loader::load(complete, shoes, error, sizeof(error)),
                           mj_deleteModel);
        require(!incompatible && std::string(error).find("requires a scene") != std::string::npos,
                "Model selection must reject a file without a scene model include");
        bool rejected = false;
        try {
            model_loader::resolve_model("missing.xml", dir);
        } catch (const std::exception&) {
            rejected = true;
        }
        require(rejected, "Missing model paths must be rejected");
        require(model_loader::escape_xml("a&'\"<>") == "a&amp;&apos;&quot;&lt;&gt;",
                "Model include paths are not XML escaped");
        std::cout << "G1 model geometry, DDS ordering, 12 terrain combinations, physics, and selection checks passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
