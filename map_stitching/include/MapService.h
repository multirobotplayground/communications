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

 #include "rclcpp/rclcpp.hpp"
 #include "comm_msgs/srv/occ_transfer.hpp"
 #include "nav_msgs/msg/occupancy_grid.hpp"
 #include "string.h"
 #include <vector>

 class MapService : public rclcpp::Node {
    public:
        MapService() : Node("MapService") {
            declare_parameter<int>("queue_size", 10);
            get_parameter("queue_size", aQueueSize);
            aNamespace = get_namespace();

            aSubscribers.push_back(
                create_subscription<nav_msgs::msg::OccupancyGrid>(
                    aNamespace + "/map", 
                    aQueueSize, 
                    std::bind(&MapService::OccCallback, this, std::placeholders::_1))
            );

            aOccService = create_service<comm_msgs::srv::OccTransfer>(
                aNamespace + "/request_map", 
                std::bind(&MapService::MapRequestServiceCallback,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2));

            aHasGrid = false;
        }

        ~MapService() {

        }

        void OccCallback(const nav_msgs::msg::OccupancyGrid msg) {
            aHasGrid = true;
            aLastReceivedGrid.data = msg.data;
            aLastReceivedGrid.header = msg.header;
            aLastReceivedGrid.info = msg.info;
        }

        void MapRequestServiceCallback(const std::shared_ptr<comm_msgs::srv::OccTransfer::Request> request,
                                             std::shared_ptr<comm_msgs::srv::OccTransfer::Response> response) {
            if(!aHasGrid) {
                response->message = "I do not have a map yet!";
                response->success = false;
                return;
            }
            
            response->message = "Here is your map!";
            response->success = true;
            response->requested_grid.data = aLastReceivedGrid.data;
            response->requested_grid.header = aLastReceivedGrid.header;
            response->requested_grid.info = aLastReceivedGrid.info;
        }
    
    private:
        std::vector<rclcpp::SubscriptionBase::SharedPtr> aSubscribers;
        rclcpp::Service<comm_msgs::srv::OccTransfer>::SharedPtr aOccService;
        rclcpp::TimerBase::SharedPtr aTimer;
        nav_msgs::msg::OccupancyGrid aLastReceivedGrid;
        bool aHasGrid;
        std::string aNamespace;
        int aQueueSize;
 };