#ifndef GAZEBO_MODEL_ATTACHMENT_PLUGIN_H
#define GAZEBO_MODEL_ATTACHMENT_PLUGIN_H

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
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

    bool attachCallback(const std::shared_ptr<gazebo_model_attachment_plugin::srv::Attach::Request> req,
                        std::shared_ptr<gazebo_model_attachment_plugin::srv::Attach::Response> res);
    bool detachCallback(const std::shared_ptr<gazebo_model_attachment_plugin::srv::Detach::Request> req,
                        std::shared_ptr<gazebo_model_attachment_plugin::srv::Detach::Response> res);

    void attach(const std::string& joint_name, physics::ModelPtr m1, physics::ModelPtr m2, physics::LinkPtr l1,
                physics::LinkPtr l2);
    void detach(const std::string& joint_name, physics::ModelPtr m1, physics::ModelPtr m2);

    gazebo_ros::Node::SharedPtr node_;

    rclcpp::Service<gazebo_model_attachment_plugin::srv::Attach>::SharedPtr attach_srv_;
    rclcpp::Service<gazebo_model_attachment_plugin::srv::Detach>::SharedPtr detach_srv_;
};
}  // namespace gazebo

#endif
