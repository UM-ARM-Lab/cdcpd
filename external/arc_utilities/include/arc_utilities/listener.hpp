#pragma once

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/spinner.h>

#include <mutex>

template <class T>
class Listener {
 protected:
  std::string topic_;
  ros::CallbackQueue queue_;
  ros::AsyncSpinner spinner_;
  ros::Subscriber sub_;
  std::mutex mtx_;
  typename T::ConstPtr msg_;

  void callback(typename T::ConstPtr const& msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    msg_ = msg;
  }

 public:
  Listener(ros::NodeHandle& nh, std::string const& topic, bool const autostart = true)
      : topic_(topic), queue_(), spinner_(1, &queue_), msg_(nullptr) {
    auto options =
        ros::SubscribeOptions::create<T>(topic, 1, boost::bind(&Listener::callback, this, _1), ros::VoidPtr(), &queue_);
    sub_ = nh.subscribe(options);
    if (autostart) {
      spinner_.start();
    }
  }

  void start() { spinner_.start(); }

  void stop() { spinner_.stop(); }

  typename T::ConstPtr get() {
    std::lock_guard<std::mutex> lock(mtx_);
    return msg_;
  }

  typename T::ConstPtr waitForNew(double const loop_freq) {
    // Clear any existing message
    {
      std::lock_guard<std::mutex> lock(mtx_);
      msg_ = nullptr;
    }
    ros::Rate r(loop_freq);
    while (ros::ok()) {
      auto const msg = get();
      if (msg) {
        return msg;
      }
      r.sleep();
    }
    throw std::runtime_error("Interrupted by !ros::ok()");
  }
};