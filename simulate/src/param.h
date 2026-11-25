#pragma once

#include <array>
#include <iostream>
#include <cmath>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include "camera_height_map.h"

namespace param
{

inline struct SimulationConfig
{
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;
    double motor_torque_scale = 1.0;
    // Unitree motor order repeats hip, thigh, calf for each leg.
    std::array<double, 3> joint_torque_scales{1.0, 1.0, 1.0};

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int show_base_velocity = 0;
    bool headless = false;
    double duration = 0;
    bool no_joystick = false;
    bool policy_height_map = true;
    bool show_policy_height_map = true;
    bool forward_elevation = false;
    bool shift_elevation_forward = true;
    float elevation_forward_offset = 0.3f;
    CameraHeightMapConfig camera_height_map;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            motor_torque_scale = cfg["motor_torque_scale"].as<double>(1.0);
            if (!std::isfinite(motor_torque_scale) || motor_torque_scale < 0 || motor_torque_scale > 1)
                throw std::invalid_argument("motor_torque_scale must be between 0 and 1");
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();

            if (const auto node = cfg["show_base_velocity"])
            {
                show_base_velocity = node.as<int>();
            }
            policy_height_map = cfg["policy_height_map"].as<bool>(true);
            show_policy_height_map = cfg["show_policy_height_map"].as<bool>(true);
            forward_elevation = cfg["forward_elevation"].as<bool>(false);
            shift_elevation_forward = cfg["shift_elevation_forward"].as<bool>(true);
            elevation_forward_offset = cfg["elevation_forward_offset"].as<float>(0.3f);
            camera_height_map.load(cfg["camera_height_map"]);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Go2 MuJoCo");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
        ("hip-torque-scale", po::value<double>(&config.joint_torque_scales[0]), "Hip abduction torque multiplier on all four legs (0 to 1, default 1)")
        ("thigh-torque-scale", po::value<double>(&config.joint_torque_scales[1]), "Thigh pitch torque multiplier on all four legs (0 to 1, default 1)")
        ("calf-torque-scale", po::value<double>(&config.joint_torque_scales[2]), "Knee torque multiplier on all four legs (0 to 1, default 1)")
        ("headless", po::bool_switch(&config.headless), "Run without a viewer")
        ("duration", po::value<double>(&config.duration), "Stop after this many simulation seconds; 0 is unlimited")
        ("no-joystick", po::bool_switch(&config.no_joystick), "Disable the physical joystick")
        ("show-policy-height-map", po::value<bool>(&config.show_policy_height_map), "Show sampled terrain points; true or false")
        ("shift-elevation-forward", po::value<bool>(&config.shift_elevation_forward), "Enable the configured elevation forward offset; true (default) or false")
        ("elevation-forward-offset", po::value<float>(&config.elevation_forward_offset), "Elevation grid forward offset in metres; default 0.3")
        ("forward-elevation", po::value<bool>(&config.forward_elevation), "Publish the forward 9x11 map instead of the full 17x11 map; true or false")
        ("camera-height-map", po::value<bool>(&config.camera_height_map.enabled)->implicit_value(true), "Use Isaac Lab camera visibility, memory and observations; true or false")
        ("camera-height-map-debug", po::value<bool>(&config.camera_height_map.debug)->implicit_value(true), "Draw Isaac Lab height-map layers and camera range (default true)")
        ("camera-height-map-augmentation", po::value<bool>(&config.camera_height_map.training_augmentation)->implicit_value(true), "Use training noise, delay, map errors and full-view episodes (default true)")
    ;

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (const po::error& error) {
        std::cerr << error.what() << '\n';
        exit(EXIT_FAILURE);
    }
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    for (double scale : config.joint_torque_scales) {
        if (!(scale >= 0 && scale <= 1)) {
            std::cerr << "Joint torque multipliers must be between 0 and 1\n";
            exit(EXIT_FAILURE);
        }
    }
    return vm;
}
}
