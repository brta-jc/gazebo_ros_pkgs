// Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the company nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gazebo/common/Plugin.hh>
#include <gazebo_plugins/gazebo_ros_model_attachment.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_msgs/srv/attach.hpp>
#include <gazebo_msgs/srv/detach.hpp>

#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>
#include <string>
#include <vector>

namespace gazebo
{

    ModelAttachmentPlugin::ModelAttachmentPlugin()
    {
    }

    ModelAttachmentPlugin::~ModelAttachmentPlugin()
    {
    }

    // cppcheck-suppress unusedFunction
    void ModelAttachmentPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
    {
        world_ = world;

        node_ = gazebo_ros::Node::Get(sdf);

        attach_srv_ = node_->create_service<gazebo_msgs::srv::Attach>(
            "/gazebo/attach",
            std::bind(&ModelAttachmentPlugin::attachCallback, this, std::placeholders::_1, std::placeholders::_2));
        detach_srv_ = node_->create_service<gazebo_msgs::srv::Detach>(
            "/gazebo/detach",
            std::bind(&ModelAttachmentPlugin::detachCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

    bool ModelAttachmentPlugin::attachCallback(
        const std::shared_ptr<gazebo_msgs::srv::Attach::Request> req,
        std::shared_ptr<gazebo_msgs::srv::Attach::Response> res)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                           "Received request to attach model: '" << req->model_name_1 << "' to '" << req->model_name_2);

        // block any other physics pose updates
        boost::recursive_mutex::scoped_lock plock(*(world_->Physics()->GetPhysicsUpdateMutex()));

        const std::string &model_1_name = req->model_name_1;
        const std::string &model_2_name = req->model_name_2;
        const gazebo::physics::Model_V models = world_->Models();

        auto m1 =
            std::find_if(models.begin(), models.end(),
                         [&model_1_name](const gazebo::physics::ModelPtr &ptr)
                         { return ptr->GetName() == model_1_name; });
        if (m1 == models.end())
        {
            const std::string error_msg = "Could not find model " + req->model_name_1;
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), error_msg);
            res->message = error_msg;
            res->success = false;
            return true;
        }

        auto m2 =
            std::find_if(models.begin(), models.end(),
                         [&model_2_name](const gazebo::physics::ModelPtr &ptr)
                         { return ptr->GetName() == model_2_name; });
        if (m2 == models.end())
        {
            const std::string error_msg = "Could not find model " + req->model_name_2;
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), error_msg);
            res->message = error_msg;
            res->success = false;
            return true;
        }

        physics::LinkPtr l1 = (*m1)->GetLink(req->link_name_1);
        if (l1 == nullptr)
        {
            const std::string error_msg = "Could not find link " + req->link_name_1 + " on model " + req->model_name_1;
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), error_msg);
            res->message = error_msg;
            res->success = false;
            return true;
        }

        physics::LinkPtr l2 = (*m2)->GetLink(req->link_name_2);
        if (l2 == nullptr)
        {
            const std::string error_msg = "Could not find link " + req->link_name_2 + " on model " + req->model_name_2;
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), error_msg);
            res->message = error_msg;
            res->success = false;
            return true;
        }

        try
        {
            attach(req->joint_name, *m1, *m2, l1, l2);
        }
        catch (const std::exception &e)
        {
            const std::string error_msg = "Failed to detach: " + std::string(e.what());
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), error_msg);
            res->message = error_msg;
            res->success = false;
            return true;
        }

        res->success = true;
        return true;
    }

    // cppcheck-suppress constParameterCallback
    bool ModelAttachmentPlugin::detachCallback(
        const std::shared_ptr<gazebo_msgs::srv::Detach::Request> req,
        std::shared_ptr<gazebo_msgs::srv::Detach::Response> res)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                           "Received request to detach model: '" << req->model_name_1 << "' from '" << req->model_name_2);

        const std::string &model_1_name = req->model_name_1;
        const std::string &model_2_name = req->model_name_2;
        const gazebo::physics::Model_V models = world_->Models();

        auto m1 =
            std::find_if(models.begin(), models.end(),
                         [&model_1_name](const gazebo::physics::ModelPtr &ptr)
                         { return ptr->GetName() == model_1_name; });
        if (m1 == models.end())
        {
            const std::string error_msg = "Could not find model " + req->model_name_1;
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), error_msg);
            res->message = error_msg;
            res->success = false;
            return true;
        }

        auto m2 =
            std::find_if(models.begin(), models.end(),
                         [&model_2_name](const gazebo::physics::ModelPtr &ptr)
                         { return ptr->GetName() == model_2_name; });
        if (m2 == models.end())
        {
            const std::string error_msg = "Could not find model " + req->model_name_2;
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), error_msg);
            res->message = error_msg;
            res->success = false;
            return true;
        }

        try
        {
            detach(req->joint_name, *m1, *m2);
        }
        catch (const std::exception &e)
        {
            const std::string error_msg = "Failed to detach: " + std::string(e.what());
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), error_msg);
            res->message = error_msg;
            res->success = false;
            return true;
        }

        res->success = true;
        return true;
    }

    void ModelAttachmentPlugin::attach(const std::string &joint_name, physics::ModelPtr m1, physics::ModelPtr m2,
                                       physics::LinkPtr l1, physics::LinkPtr l2)
    {
        if (m1 == nullptr)
            throw std::runtime_error("Model 1 is null");

        if (m2 == nullptr)
            throw std::runtime_error("Model 2 is null");

        if (l1 == nullptr)
            throw std::runtime_error("Link 1 is null");

        if (l2 == nullptr)
            throw std::runtime_error("Link 2 is null");

        ignition::math::Pose3d m1wp = m1->WorldPose();
        ignition::math::Pose3d l1rl = l1->RelativePose();
        ignition::math::Pose3d l2rl = l2->RelativePose();
        ignition::math::Pose3d p = (m1wp * l1rl * l2rl.Inverse());

        m2->SetWorldPose(p);

        physics::JointPtr joint = m1->CreateJoint(joint_name, "fixed", l1, l2);

        if (joint == nullptr)
            throw std::runtime_error("CreateJoint returned nullptr");

        m1->AddChild(m2);
    }

    void ModelAttachmentPlugin::detach(const std::string &joint_name, physics::ModelPtr m1, physics::ModelPtr m2)
    {
        if (m1 == nullptr)
            throw std::runtime_error("Model 1 is null");

        if (m2 == nullptr)
            throw std::runtime_error("Model 2 is null");

        physics::JointPtr joint = m1->GetJoint(joint_name);
        if (joint == nullptr)
            throw std::runtime_error("No joint on model " + m1->GetName() + " by name " + joint_name);

        bool success = m1->RemoveJoint(joint_name);

        if (!success)
            throw std::runtime_error("Unable to remove joint from model");

        m2->SetParent(m1->GetWorld()->ModelByName("default"));

        // We need to flush the children vector of the parent
        // Calling m1->RemoveChild(boost::dynamic_pointer_cast<physics::Entity>(m2)); will also destroy the child
        physics::Base_V temp_child_objects;
        unsigned int children_count = m1->GetChildCount();
        for (unsigned int i = 0; i < children_count; i++)
        {
            if (m1->GetChild(i) != m2)
                temp_child_objects.push_back(m1->GetChild(i));
        }

        m1->RemoveChildren();

        for (const auto &obj : temp_child_objects)
        {
            m1->AddChild(obj);
        }

        return;
    }

    GZ_REGISTER_WORLD_PLUGIN(ModelAttachmentPlugin)
} // namespace gazebo
