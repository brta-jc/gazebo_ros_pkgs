#ifndef gazebo_msgs_H
#define gazebo_msgs_H

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_msgs/srv/attach.hpp>
#include <gazebo_msgs/srv/detach.hpp>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gazebo_ros/node.hpp"

namespace gazebo
{

  class ModelAttachmentPlugin : public WorldPlugin
  {
  public:
    ModelAttachmentPlugin();

    virtual ~ModelAttachmentPlugin();

  protected:
    void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

  private:
    physics::WorldPtr world_;

    bool attachCallback(const std::shared_ptr<gazebo_msgs::srv::Attach::Request> req,
                        std::shared_ptr<gazebo_msgs::srv::Attach::Response> res);
    bool detachCallback(const std::shared_ptr<gazebo_msgs::srv::Detach::Request> req,
                        std::shared_ptr<gazebo_msgs::srv::Detach::Response> res);

    void attach(const std::string &joint_name, physics::ModelPtr m1, physics::ModelPtr m2, physics::LinkPtr l1,
                physics::LinkPtr l2);
    void detach(const std::string &joint_name, physics::ModelPtr m1, physics::ModelPtr m2);

    gazebo_ros::Node::SharedPtr node_;

    rclcpp::Service<gazebo_msgs::srv::Attach>::SharedPtr attach_srv_;
    rclcpp::Service<gazebo_msgs::srv::Detach>::SharedPtr detach_srv_;
  };
} // namespace gazebo

#endif
