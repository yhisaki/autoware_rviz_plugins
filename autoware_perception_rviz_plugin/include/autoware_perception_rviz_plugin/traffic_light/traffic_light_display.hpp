//
//  Copyright 2025 TIER IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef AUTOWARE_PERCEPTION_RVIZ_PLUGIN__TRAFFIC_LIGHT__TRAFFIC_LIGHT_DISPLAY_HPP_
#define AUTOWARE_PERCEPTION_RVIZ_PLUGIN__TRAFFIC_LIGHT__TRAFFIC_LIGHT_DISPLAY_HPP_

#include "autoware_perception_rviz_plugin/common/text_object.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_rendering/objects/shape.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <boost/optional/optional.hpp>

#include <OgreSceneNode.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware_perception_rviz_plugin::traffic_light
{

struct Point3d
{
  double x;
  double y;
  double z;
};

struct TrafficLightBulbInfo
{
  lanelet::Id id;
  std::string color;
  std::string shape;
  Point3d position;
};

struct TrafficLightInfo
{
  lanelet::Id id;
  lanelet::ConstLineString3d linestring;
  boost::optional<double> height;
  std::vector<TrafficLightBulbInfo> bulbs;

  [[nodiscard]] TrafficLightBulbInfo getEstimatedBulb(const std::string & color) const;

  [[nodiscard]] Point3d getLinestringCenter() const;

  [[nodiscard]] std::vector<TrafficLightBulbInfo> getBlightBulbs(
    const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & current_elements) const;
};

class TrafficLightDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  TrafficLightDisplay();
  ~TrafficLightDisplay() override;

protected:
  void onInitialize() override;
  void reset() override;
  void update(float wall_dt, float ros_dt) override;
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void topic_updated_lanelet_map();
  void topic_updated_traffic_light();

private:  // NOLINT
  // Display methods
  void hideAllDisplays();
  void updateTrafficLightText(const TrafficLightInfo & info, const std::string & state_text);
  void updateTrafficLightBulbs(
    const TrafficLightInfo & info,
    const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & elements);
  bool checkTimeout() const;

  // Properties
  std::unique_ptr<rviz_common::properties::RosTopicProperty> lanelet_map_topic_property_;
  std::unique_ptr<rviz_common::properties::RosTopicProperty> traffic_light_topic_property_;
  std::unique_ptr<rviz_common::properties::FloatProperty> timeout_property_;
  std::unique_ptr<rviz_common::properties::FloatProperty> text_z_offset_property_;
  std::unique_ptr<rviz_common::properties::FloatProperty> text_x_offset_property_;
  std::unique_ptr<rviz_common::properties::FloatProperty> text_y_offset_property_;
  std::unique_ptr<rviz_common::properties::BoolProperty> show_text_property_;
  std::unique_ptr<rviz_common::properties::BoolProperty> show_bulb_property_;
  std::unique_ptr<rviz_common::properties::StringProperty> text_prefix_property_;
  std::unique_ptr<rviz_common::properties::FloatProperty> font_size_property_;
  std::unique_ptr<rviz_common::properties::ColorProperty> text_color_property_;
  std::unique_ptr<rviz_common::properties::FloatProperty> bulb_radius_property_;

  // Subscribers
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr lanelet_map_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>::SharedPtr
    traffic_light_group_array_sub_;

  // Callback functions
  void onLaneletMapReceived(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  void onTrafficLightGroupArrayReceived(
    const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr msg);

  std::mutex property_mutex_;
  std::mutex lanelet_map_mutex_;
  std::mutex traffic_light_mutex_;
  void setupRosSubscriptions();

  // Data storage
  lanelet::LaneletMapPtr lanelet_map_;
  std_msgs::msg::Header lanelet_map_header_;

  autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr traffic_light_groups_;
  rclcpp::Time last_traffic_light_received_time_;

  // Text visualization
  std::unordered_map<lanelet::Id, std::unique_ptr<common::TextObject>> traffic_light_text_displays_;

  // Shape visualization
  std::unordered_map<lanelet::Id, std::unique_ptr<rviz_rendering::Shape>>
    traffic_light_bulb_displays_;
};

}  // namespace autoware_perception_rviz_plugin::traffic_light

#endif  // AUTOWARE_PERCEPTION_RVIZ_PLUGIN__TRAFFIC_LIGHT__TRAFFIC_LIGHT_DISPLAY_HPP_
