#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>

#include <mujoco/mujoco.h>
#include <unitree/common/thread/recurrent_thread.hpp>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include "param.h"
#include "physics_joystick.h"

class G1Bridge
{
    using LowCmd = unitree::robot::g1::subscription::LowCmd;
    using LowState = unitree::robot::g1::publisher::LowState;
    // Shared telemetry IDL; G1 motor commands and states use unitree_hg.
    using HighState = unitree::robot::RealTimePublisher<unitree_go::msg::dds_::SportModeState_>;
    using WirelessController = unitree::robot::RealTimePublisher<unitree_go::msg::dds_::WirelessController_>;
    using BmsState = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;

public:
    G1Bridge(mjModel *model, mjData *data) : model_(model), data_(data)
    {
        if (model_->nu != 29 || model_->nq != 36 || model_->nv != 35 ||
            model_->nsensordata < 3 * model_->nu ||
            mj_name2id(model_, mjOBJ_BODY, "pelvis") < 0 ||
            mj_name2id(model_, mjOBJ_BODY, "torso_link") < 0)
        {
            throw std::runtime_error("Expected a floating-base G1 29-DOF model with 29 motor/sensor slots");
        }

        if (param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if (param::config.use_joystick == 1) {
            if (param::config.joystick_type == "xbox") {
                joystick_ = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if (param::config.joystick_type == "switch") {
                joystick_ = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                throw std::runtime_error("Unsupported joystick type: " + param::config.joystick_type);
            }
        }

        imu_ = {sensorAddress("imu_quat"), sensorAddress("imu_gyro"), sensorAddress("imu_acc")};
        secondary_imu_ = {sensorAddress("secondary_imu_quat"), sensorAddress("secondary_imu_gyro"), sensorAddress("secondary_imu_acc")};
        frame_pos_ = sensorAddress("frame_pos");
        frame_vel_ = sensorAddress("frame_vel");

        lowcmd_ = std::make_shared<LowCmd>("rt/lowcmd");
        lowstate_ = std::make_unique<LowState>();
        lowstate_->joystick = joystick_;
        lowstate_->msg_.mode_machine() = 5;
        highstate_ = std::make_unique<HighState>("rt/sportmodestate");
        wireless_controller_ = std::make_unique<WirelessController>("rt/wirelesscontroller");
        bmsstate_ = std::make_unique<BmsState>("rt/lf/bmsstate");
        bmsstate_->msg_.soc() = 100;
        secondary_imustate_ = std::make_unique<IMUState>("rt/secondary_imu");
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "g1_bridge", UT_CPU_ID_NONE, 1000, [this]() { run(); });
    }

    void run()
    {
        if (joystick_) {
            joystick_->update();
        }
        const int num_motor = model_->nu;
        {
            std::lock_guard<std::mutex> lock(lowcmd_->mutex_);
            for (int i = 0; i < num_motor; ++i) {
                const auto &cmd = lowcmd_->msg_.motor_cmd()[i];
                data_->ctrl[i] = cmd.tau() +
                    cmd.kp() * (cmd.q() - data_->sensordata[i]) +
                    cmd.kd() * (cmd.dq() - data_->sensordata[i + num_motor]);
            }
        }

        if (lowstate_->trylock()) {
            for (int i = 0; i < num_motor; ++i) {
                auto &state = lowstate_->msg_.motor_state()[i];
                state.q() = data_->sensordata[i];
                state.dq() = data_->sensordata[i + num_motor];
                state.tau_est() = data_->sensordata[i + 2 * num_motor];
            }
            writeImu(lowstate_->msg_.imu_state(), imu_);
            lowstate_->msg_.tick() = std::round(data_->time / 1e-3);
            lowstate_->unlockAndPublish();
        }

        if (highstate_->trylock()) {
            for (int i = 0; i < 3; ++i) {
                if (frame_pos_ >= 0) {
                    highstate_->msg_.position()[i] = data_->sensordata[frame_pos_ + i];
                }
                if (frame_vel_ >= 0) {
                    highstate_->msg_.velocity()[i] = data_->sensordata[frame_vel_ + i];
                }
            }
            highstate_->unlockAndPublish();
        }

        if (joystick_ && wireless_controller_->trylock()) {
            auto &msg = wireless_controller_->msg_;
            msg.lx() = joystick_->lx();
            msg.ly() = joystick_->ly();
            msg.rx() = joystick_->rx();
            msg.ry() = joystick_->ry();
            msg.keys() = joystick_->combine().RF_RX.btn.value;
            wireless_controller_->unlockAndPublish();
        }
        if (secondary_imustate_->trylock()) {
            writeImu(secondary_imustate_->msg_, secondary_imu_);
            secondary_imustate_->unlockAndPublish();
        }
        if (bmsstate_->trylock()) {
            bmsstate_->unlockAndPublish();
        }
    }

private:
    struct ImuAddresses
    {
        int quat;
        int gyro;
        int acc;
    };

    int sensorAddress(const char *name) const
    {
        const int id = mj_name2id(model_, mjOBJ_SENSOR, name);
        return id >= 0 ? model_->sensor_adr[id] : -1;
    }

    template <typename ImuMessage>
    void writeImu(ImuMessage &msg, const ImuAddresses &adr) const
    {
        if (adr.quat >= 0) {
            for (int i = 0; i < 4; ++i) {
                msg.quaternion()[i] = data_->sensordata[adr.quat + i];
            }
            const double w = msg.quaternion()[0];
            const double x = msg.quaternion()[1];
            const double y = msg.quaternion()[2];
            const double z = msg.quaternion()[3];
            msg.rpy()[0] = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
            msg.rpy()[1] = std::asin(std::clamp(2 * (w * y - z * x), -1.0, 1.0));
            msg.rpy()[2] = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        }
        for (int i = 0; i < 3; ++i) {
            if (adr.gyro >= 0) {
                msg.gyroscope()[i] = data_->sensordata[adr.gyro + i];
            }
            if (adr.acc >= 0) {
                msg.accelerometer()[i] = data_->sensordata[adr.acc + i];
            }
        }
    }

    void printSceneInformation() const
    {
        auto printObjects = [this](const char *title, int count, mjtObj type) {
            std::cout << "<<------------- " << title << " ------------->>\n";
            for (int i = 0; i < count; ++i) {
                if (const char *name = mj_id2name(model_, type, i)) {
                    const int index = type == mjOBJ_SENSOR ? model_->sensor_adr[i] : i;
                    std::cout << title << "_index: " << index << ", name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << model_->sensor_dim[i];
                    }
                    std::cout << '\n';
                }
            }
            std::cout << '\n';
        };
        printObjects("Link", model_->nbody, mjOBJ_BODY);
        printObjects("Joint", model_->njnt, mjOBJ_JOINT);
        printObjects("Actuator", model_->nu, mjOBJ_ACTUATOR);
        printObjects("Sensor", model_->nsensor, mjOBJ_SENSOR);
    }

    mjModel *model_;
    mjData *data_;
    ImuAddresses imu_;
    ImuAddresses secondary_imu_;
    int frame_pos_;
    int frame_vel_;
    std::shared_ptr<unitree::common::UnitreeJoystick> joystick_;
    std::shared_ptr<LowCmd> lowcmd_;
    std::unique_ptr<LowState> lowstate_;
    std::unique_ptr<HighState> highstate_;
    std::unique_ptr<WirelessController> wireless_controller_;
    std::unique_ptr<BmsState> bmsstate_;
    std::unique_ptr<IMUState> secondary_imustate_;
    unitree::common::RecurrentThreadPtr thread_;
};
