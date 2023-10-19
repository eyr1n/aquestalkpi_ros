#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <SDL2/SDL.h>
#include <SDL2/SDL_mixer.h>
#include <rclcpp/rclcpp.hpp>

#include "aquestalkpi.hpp"
#include "aquestalkpi_ros_msgs/msg/talk.hpp"

namespace aquestalkpi_ros {

class AquesTalkPiRosComponent : public rclcpp::Node {
public:
  AquesTalkPiRosComponent(const rclcpp::NodeOptions &options)
      : AquesTalkPiRosComponent("", options) {}

  AquesTalkPiRosComponent(
      const std::string &name_space = "",
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("aquestalkpi_ros_node", name_space, options) {
    // パラメータ取得
    declare_parameter("aquestalkpi_path", "./aquestalkpi/AquesTalkPi");
    auto aquestalkpi_path = get_parameter("aquestalkpi_path").as_string();

    // AquesTalkPi初期化
    aquestalkpi_ = std::make_shared<AquesTalkPi>(aquestalkpi_path);

    // SDL2初期化
    SDL_Init(SDL_INIT_AUDIO);
    Mix_Init(0);
    Mix_OpenAudio(8000, AUDIO_S16LSB, 1, 4096);
    Mix_ChannelFinished([](int channel) { Mix_FreeChunk(channels_[channel]); });

    // サブスクライバ
    aquestalkpi_ros_sub_ =
        this->create_subscription<aquestalkpi_ros_msgs::msg::Talk>(
            "aquestalkpi_ros", rclcpp::QoS(10),
            std::bind(&AquesTalkPiRosComponent::aquestalkpi_ros_callback, this,
                      std::placeholders::_1));
  }

  ~AquesTalkPiRosComponent() {
    Mix_HaltChannel(-1);
    Mix_CloseAudio();
    Mix_Quit();
    SDL_Quit();
  }

private:
  static inline std::unordered_map<int, Mix_Chunk *> channels_;
  std::vector<uint8_t> wav_buf_;
  std::string err_buf_;

  std::shared_ptr<AquesTalkPi> aquestalkpi_;
  rclcpp::Subscription<aquestalkpi_ros_msgs::msg::Talk>::SharedPtr
      aquestalkpi_ros_sub_;

  void aquestalkpi_ros_callback(
      const aquestalkpi_ros_msgs::msg::Talk::SharedPtr msg) {
    aquestalkpi_->synthesize(msg->text, msg->voice, msg->speed, wav_buf_,
                             err_buf_);
    if (err_buf_.empty()) {
      Mix_HaltChannel(-1);
      Mix_Chunk *chunk = Mix_QuickLoad_WAV(wav_buf_.data());
      channels_[Mix_PlayChannel(-1, chunk, 0)] = chunk;
      RCLCPP_INFO(this->get_logger(), "%s", msg->text.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), err_buf_.c_str());
    }
  }
};

} // namespace aquestalkpi_ros
