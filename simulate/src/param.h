#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <stdexcept>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    bool enable_camera = false;
    std::filesystem::path camera_model = "d435i.xml";
    std::string camera_name = "front_depth_camera";
    std::string camera_topic = "rt/front_depth_observation";
    int camera_width = 640;
    int camera_height = 360;
    double camera_publish_hz = 15.0;
    bool show_depth = true;
    double depth_display_max = 2.0;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();

            if (const auto camera = cfg["camera"])
            {
                if (camera["enabled"]) enable_camera = camera["enabled"].as<bool>();
                if (camera["model"]) camera_model = camera["model"].as<std::string>();
                if (camera["name"]) camera_name = camera["name"].as<std::string>();
                if (camera["topic"]) camera_topic = camera["topic"].as<std::string>();
                if (camera["width"]) camera_width = camera["width"].as<int>();
                if (camera["height"]) camera_height = camera["height"].as<int>();
                if (camera["publish_hz"]) camera_publish_hz = camera["publish_hz"].as<double>();
                if (camera["show_depth"]) show_depth = camera["show_depth"].as<bool>();
                if (camera["display_max_depth"]) depth_display_max = camera["display_max_depth"].as<double>();
            }

            if (camera_width <= 0 || camera_height <= 0 || camera_publish_hz <= 0.0 ||
                depth_display_max <= 0.0)
            {
                throw std::runtime_error("camera width, height, publish_hz and display_max_depth must be positive");
            }
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
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}
