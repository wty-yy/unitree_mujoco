#pragma once

#include <iostream>
#include <cmath>
#include <stdexcept>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace param
{

inline struct SimulationConfig
{
    std::filesystem::path robot_scene;
    std::filesystem::path robot_model;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;
    int show_base_velocity = 1;
    bool show_policy_height_map = true;
    double height_map_offset_x = 0.5;
    double waist_pitch_torque_scale = 1.0;

    int enable_elastic_band;
    int band_attached_link = -1;

    void load_from_yaml(const std::string &filename)
    {
        try
        {
            auto cfg = YAML::LoadFile(filename);
            robot_scene = cfg["robot_scene"].as<std::string>();
            robot_model = cfg["robot_model"] ? cfg["robot_model"].as<std::string>() : "default";
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            show_base_velocity = cfg["show_base_velocity"].as<int>(1);
            show_policy_height_map = cfg["show_policy_height_map"].as<bool>(true);
            height_map_offset_x = cfg["height_map_offset_x"].as<double>(0.5);
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();
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
    po::options_description desc("Unitree G1 MuJoCo");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "G1 scene file; -s scene_29dof.xml")
        ("model,m", po::value<std::filesystem::path>(&config.robot_model), "G1 model: default, complete, or model XML path")
        ("waist-pitch-torque-scale", po::value<double>(&config.waist_pitch_torque_scale), "Final waist-pitch motor torque multiplier in [0, 1]; default 1.0")
        ("show-policy-height-map", po::value<bool>(&config.show_policy_height_map), "Show sampled terrain points; true or false")
        ("height-map-offset-x", po::value<double>(&config.height_map_offset_x), "Height-map grid forward offset in metres; default 0.5")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    if (!std::isfinite(config.waist_pitch_torque_scale) ||
        config.waist_pitch_torque_scale < 0.0 || config.waist_pitch_torque_scale > 1.0) {
        throw std::invalid_argument("--waist-pitch-torque-scale must be finite and in [0, 1]");
    }

    if (!std::isfinite(config.height_map_offset_x)) {
        throw std::invalid_argument("--height-map-offset-x must be finite");
    }

    return vm;
}

}
