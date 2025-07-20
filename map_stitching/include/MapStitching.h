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
#include "string.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "comm_msgs/msg/comm_event.hpp"
#include "comm_msgs/srv/occ_transfer.hpp"
#include <cmath>
#include <vector>
#include <boost/algorithm/string.hpp>


 class MapStitching : public rclcpp::Node {
    public:
        MapStitching() : rclcpp::Node("MapStitching") {
            declare_parameter<int>("queue_size", 10);
            declare_parameter<double>("update_frequency_hz", 5.0);
            declare_parameter<double>("map_waiting_timeout", 30.0);
            declare_parameter<int>("robots", 3);
            get_parameter("queue_size", aQueueSize);
            get_parameter("update_frequency_hz", aFrequency);
            get_parameter("map_waiting_timeout", aMapWaitTimeout);
            get_parameter("robots", aRobots);

            aNamespace = get_namespace();
            std::vector<std::string> splitted;
            boost::split(splitted, aNamespace, boost::is_any_of("_"));
            aId = atoi(splitted[splitted.size()-1].c_str());
            aReceivedMaps.assign(aRobots, false);
            aDirtyArray.assign(aRobots, false);

            aInitialized = false;

            for(int i = 0; i < aRobots; ++i) aMaps.push_back(nav_msgs::msg::OccupancyGrid());

            double update_period = 1.0 / aFrequency;
            apTimer = create_wall_timer(std::chrono::duration<double>(update_period), 
                                        std::bind(&MapStitching::Update, this));

            aSubscribers.push_back(
                create_subscription<comm_msgs::msg::CommEvent>
                    (aNamespace + "/comm_event",
                    aQueueSize,
                    std::bind(&MapStitching::CommEventCallback, this, std::placeholders::_1))
                );

            aSubscribers.push_back(
                create_subscription<nav_msgs::msg::OccupancyGrid>(
                    aNamespace + "/map", 
                    aQueueSize, 
                    std::bind(&MapStitching::OccCallback, this, std::placeholders::_1))
                );

            apOccPub = create_publisher<nav_msgs::msg::OccupancyGrid>(aNamespace + "/merged_map", aQueueSize);
        }

        ~MapStitching() {

        }

        void OccCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            aReceivedMaps[aId-1] = true;
            aDirtyArray[aId-1] = true;
            aMaps[aId-1].data = msg->data;
            aMaps[aId-1].header = msg->header;
            aMaps[aId-1].info = msg->info;
        }

        void CommEventCallback(const comm_msgs::msg::CommEvent::SharedPtr msg) {
            int other_robot = msg->client;

            // create a node
            std::shared_ptr<rclcpp::Node> spin_node = rclcpp::Node::make_shared("comm_request_node", aNamespace);

            // create a service caller
            std::string client_namespace = "/robot_" + std::to_string(other_robot);
            rclcpp::Client<comm_msgs::srv::OccTransfer>::SharedPtr service = 
                spin_node->create_client<comm_msgs::srv::OccTransfer>(client_namespace + "/request_map");

            auto request = std::make_shared<comm_msgs::srv::OccTransfer::Request>();

            // wait it to be ready
            while(!service->wait_for_service(std::chrono::duration<double>(aMapWaitTimeout))) {
                if(!rclcpp::ok()) {
                    RCLCPP_INFO(get_logger(), "Failed to call service due to ROS error during the waiting service structure.");
                    return;
                }
                RCLCPP_INFO(get_logger(), "Service called correctly");
            }
            
            // spin until completion
            auto result = service->async_send_request(request);
            if(rclcpp::spin_until_future_complete(spin_node, result) == rclcpp::FutureReturnCode::SUCCESS) {
                auto response = result.get();

                // merging the received maps
                aMaps[other_robot-1].data   = response->requested_grid.data;
                aMaps[other_robot-1].header = response->requested_grid.header;
                aMaps[other_robot-1].info   = response->requested_grid.info;
                aReceivedMaps[other_robot-1] = true;
                aDirtyArray[other_robot-1] = true;
            }
        }

        void set_value(nav_msgs::msg::OccupancyGrid& grid, const int& x, const int& y, int8_t value) {
            if (x >= 0 && x < (int)grid.info.width &&
                y >= 0 && y < (int)grid.info.height) {
                    int8_t& to_replace = grid.data[y * grid.info.width + x];
                    if(to_replace == -1) to_replace = value;
            }
        }

        nav_msgs::msg::OccupancyGrid Stitch(nav_msgs::msg::OccupancyGrid& A, nav_msgs::msg::OccupancyGrid& B) {      
            nav_msgs::msg::OccupancyGrid out;
     
            // Calculate combined bounds
            double new_origin_x = std::min(A.info.origin.position.x, B.info.origin.position.x);
            double new_origin_y = std::min(A.info.origin.position.y, B.info.origin.position.y);
            double max_x = std::max(A.info.origin.position.x + A.info.width * A.info.resolution,
                                B.info.origin.position.x + B.info.width * B.info.resolution);
            double max_y = std::max(A.info.origin.position.y + A.info.height * A.info.resolution,
                                B.info.origin.position.y + B.info.height * B.info.resolution);

            
            // Set up output grid
            out.header.stamp = this->get_clock()->now();
            out.header.frame_id = aMaps[aId-1].header.frame_id;
            out.info.resolution = aMaps[aId-1].info.resolution;

            int new_width = (max_x - new_origin_x) / out.info.resolution;
            int new_height = (max_y - new_origin_y) / out.info.resolution;
            out.info.width = new_width;
            out.info.height = new_height;
            out.info.origin.position.x = new_origin_x;
            out.info.origin.position.y = new_origin_y;
            out.info.origin.position.z = 0.0;
            out.info.origin.orientation.w = 1.0;
            
            // Initialize output data with unknown values (-1)
            out.data.assign(out.info.width * out.info.height, -1);
            
            // Copy data from grid A
            int start_x = (A.info.origin.position.x - new_origin_x) / out.info.resolution;
            int start_y = (A.info.origin.position.y - new_origin_y) / out.info.resolution;

            for (uint32_t y = 0; y < A.info.height; ++y) {
                for (uint32_t x = 0; x < A.info.width; ++x) {                  
                    int8_t value = A.data[y * A.info.width + x];
                    set_value(out, start_x + x, start_y + y, value);
                }
            }
            
            // Copy data from grid B (with conflict resolution)
            start_x = (B.info.origin.position.x - new_origin_x) / out.info.resolution;
            start_y = (B.info.origin.position.y - new_origin_y) / out.info.resolution;

            for (uint32_t y = 0; y < B.info.height; ++y) {
                for (uint32_t x = 0; x < B.info.width; ++x) {
                    int8_t value = B.data[y * B.info.width + x];
                    set_value(out, start_x + x, start_y + y, value);
                }
            }

            return out;
        }

        void MergeMaps() {
            // always have a clean buffer with my map
            if(!aInitialized) {
                aMerged = aMaps[aId-1];
                aInitialized = true;
            }

            bool update = false;
            for(size_t i = 0; i < aMaps.size(); ++i) {
                 if(!aReceivedMaps[i] || static_cast<int>(i) == aId - 1) continue;
                 if(aDirtyArray[i]) {
                    aMerged = Stitch(aMerged, aMaps[i]);
                    aDirtyArray[i] = false;
                    update = true;
                 }
            }

            if(aDirtyArray[aId-1] || update) {
                aMerged = Stitch(aMerged, aMaps[aId-1]);
                aDirtyArray[aId-1] = false;
            }
        }

        void Update() {
            if(!aReceivedMaps[aId-1]) {
                RCLCPP_INFO(this->get_logger(), "My map was not received yet.");
                return;
            }

            MergeMaps();

            apOccPub->publish(aMerged);
        }

    private:
        int aQueueSize;
        int aRobots;
        int aId;
        double aFrequency;
        double aMapWaitTimeout;
        bool aInitialized;
        std::vector<bool> aDirtyArray;
        std::vector<bool> aReceivedMaps;
        std::vector<nav_msgs::msg::OccupancyGrid> aMaps;
        nav_msgs::msg::OccupancyGrid aMerged;
        std::string aNamespace;
        rclcpp::TimerBase::SharedPtr apTimer;
        std::vector<rclcpp::SubscriptionBase::SharedPtr> aSubscribers;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr apOccPub;
 };