#pragma once

#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

class PolicySelectionSubscriber
{
public:
    PolicySelectionSubscriber() : subscriber_("rt/g1/policy_selection")
    {
        subscriber_.InitChannel([this](const void* raw) {
            const auto& message = *static_cast<const std_msgs::msg::dds_::String_*>(raw);
            try {
                auto data = YAML::Load(message.data());
                if (!data.IsMap() || !data["active"]) return;
                if (!data["active"].as<bool>()) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    text_.clear();
                    return;
                }
                const auto id = data["selected_id"].as<std::size_t>();
                const auto names = data["expert_names"].as<std::vector<std::string>>();
                const auto weights = data["expert_weights"].as<std::vector<float>>();
                if (id >= names.size() || names.size() != weights.size()) return;
                for (std::size_t i = 0; i < names.size(); ++i) {
                    if (names[i].empty() || names[i].size() > 128 || names[i].find_first_of("\r\n\t") != std::string::npos ||
                        !std::isfinite(weights[i]) || weights[i] < 0.0f || weights[i] > 1.0f) return;
                }
                std::string text = "Dominant expert: " + names[id];
                for (std::size_t i = 0; i < names.size(); ++i) {
                    char percent[16];
                    std::snprintf(percent, sizeof(percent), "%.1f%%", 100.0 * weights[i]);
                    text += "\n" + names[i] + ": " + percent;
                }
                std::lock_guard<std::mutex> lock(mutex_);
                text_ = std::move(text);
                received_ = std::chrono::steady_clock::now();
            } catch (const YAML::Exception&) {
                // Ignore malformed telemetry without extending its lifetime.
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
    std::chrono::steady_clock::time_point received_{};
    unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_> subscriber_;
};
