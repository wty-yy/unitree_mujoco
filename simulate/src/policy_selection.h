#pragma once

#include <chrono>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

class PolicySelectionSubscriber
{
public:
    PolicySelectionSubscriber() : subscriber_("rt/go2/policy_selection")
    {
        subscriber_.InitChannel([this](const void* raw) {
            const auto& message = *static_cast<const std_msgs::msg::dds_::String_*>(raw);
            try {
                const auto data = YAML::Load(message.data());
                if (!data.IsMap() || !data["active"]) return;
                if (!data["active"].as<bool>()) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    text_.clear();
                    return;
                }
                if (!data["selected_id"] || !data["selected_name"]) return;
                const auto id = data["selected_id"].as<int>();
                const auto name = data["selected_name"].as<std::string>();
                if (id < 0 || name.empty() || name.size() > 128 || name.find_first_of("\r\n\t") != std::string::npos) return;
                std::ostringstream text;
                text << "Selected policy: " << id << " (" << name << ")";
                const auto names = data["expert_names"];
                const auto weights = data["router_weights"];
                if (names && weights && names.IsSequence() && weights.IsSequence() && names.size() == weights.size()) {
                    text << std::fixed << std::setprecision(1);
                    for (std::size_t i = 0; i < weights.size(); ++i) {
                        const auto label = names[i].as<std::string>();
                        const auto weight = weights[i].as<double>();
                        if (label.empty() || label.size() > 128 || label.find_first_of("\r\n\t") != std::string::npos ||
                            !std::isfinite(weight) || weight < 0 || weight > 1) return;
                        text << '\n' << label << "  " << weight * 100 << '%';
                    }
                }
                std::lock_guard<std::mutex> lock(mutex_);
                text_ = text.str();
                received_ = std::chrono::steady_clock::now();
            } catch (const YAML::Exception&) {
                // Ignore malformed telemetry without refreshing its lifetime.
            }
        }, 1);
    }

    ~PolicySelectionSubscriber() { subscriber_.CloseChannel(); }

    std::string overlay_text() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (std::chrono::steady_clock::now() - received_ > std::chrono::milliseconds(500)) return {};
        return text_;
    }

private:
    mutable std::mutex mutex_;
    std::string text_;
    std::chrono::steady_clock::time_point received_;
    unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_> subscriber_;
};
