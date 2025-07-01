/*
 * Jazzy-Multi-Robot-Sandbox for multi-robot research using ROS 2
 * Copyright (C) 2025 Alysson Ribeiro da Silva
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define OK 0

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "comm_msgs/msg/occupancy_grid_comm_event.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include <boost/algorithm/string.hpp>

#define NO_COMM 0
#define ESTABLISHED_COMM 1
#define IN_COMM 2

class TemplateNode : public rclcpp::Node {
    public:
        TemplateNode() : rclcpp::Node("template_node") {
            declare_parameter<int>("update_frequency_hz", 1);
            declare_parameter<int>("queue_size", 10);
            declare_parameter<int>("robots", 2);
            declare_parameter<double>("comm_distance", 5.0);
            get_parameter("update_frequency_hz", aUpdateFrequencyHz);
            get_parameter("queue_size", aQueueSize);
            get_parameter("robots", aRobots);
            get_parameter("comm_distance", aCommDistance);
            aNamespace = get_namespace();
            std::vector<std::string> splitted;
            boost::split(splitted, aNamespace, boost::is_any_of("_"));
            aId = atoi(splitted[splitted.size()-1].c_str());
            aReceivedPose = false;

            // update period
            double update_period = 1.0 / static_cast<double>(aUpdateFrequencyHz);
            apTimer = create_wall_timer(
                    std::chrono::duration<double>(update_period),
                    std::bind(&TemplateNode::Update, this));

            aCommStatus = std::vector<int>(aRobots, NO_COMM);
            aPoses = std::vector<tf2::Vector3>(aRobots, tf2::Vector3(0.0, 0.0, 0.0));

            int* id = &aId;
            bool* received_pose = &aReceivedPose;
            double* comm_dist = &aCommDistance;
            std::vector<int>* comm_status = &aCommStatus;
            std::vector<tf2::Vector3>* poses = &aPoses;
            
            for(int i = 1; i <= aRobots; ++i) {
                std::string robot_namespace = "/robot_" + std::to_string(i);

                aSubscribers.push_back(
                    create_subscription<geometry_msgs::msg::PoseStamped>(
                        robot_namespace + "/pose", 
                        aQueueSize,
                        [this, i, id, received_pose, comm_dist, comm_status, poses](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                            tf2::Vector3 pose = tf2::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
                            int poses_index = i-1;
                            int my_id_index = (*id) - 1;
                            poses->at(poses_index) = pose;

                            if(poses_index == my_id_index) {
                                (*received_pose) = true;
                            } else {
                                if(!(*received_pose)) return;
                                double distance = pose.distance(poses->at(my_id_index));
                                switch(comm_status->at(poses_index)) {
                                    case NO_COMM:
                                        if(distance <= (*comm_dist)) {
                                            comm_status->at(poses_index) = IN_COMM;
                                            this->EstablishedConnectionCallback(i);
                                        }
                                    break;
                                    case IN_COMM:
                                        if(distance > (*comm_dist)) {
                                            comm_status->at(poses_index) = NO_COMM;
                                            this->DisconnectCallback(i);
                                        }
                                    break;
                                }
                            }
                        }
                    )
                );
            }
        }

        ~TemplateNode() {}

        void EstablishedConnectionCallback(const int& other_id) {
            RCLCPP_INFO(this->get_logger(), "Established connection with robot_%d", other_id);
        }

        void DisconnectCallback(const int& other_id) {
            RCLCPP_INFO(this->get_logger(), "Disconnected from robot_%d", other_id);
        }

        // main update function
        // ------------------------------------------------------------------------------------------------------------------
        void Update() {
        }
        // ------------------------------------------------------------------------------------------------------------------
        
    private:
        // helps establishing the publishing frequency
        int aUpdateFrequencyHz;
        int aQueueSize;
        std::string aNamespace;

        // this calls the update function at a desired frequency
        rclcpp::TimerBase::SharedPtr apTimer;

        // I like to put all subs in the same place just to have a memory reserved to them
        std::vector<rclcpp::SubscriptionBase::SharedPtr> aSubscribers;

        // publishers must be declared, shame...
        rclcpp::Publisher<comm_msgs::msg::OccupancyGridCommEvent>::SharedPtr aOccPub;

        int aRobots;
        int aId;
        bool aReceivedPose;
        double aCommDistance;
        
        std::vector<tf2::Vector3> aPoses;
        std::vector<int> aCommStatus;
};

