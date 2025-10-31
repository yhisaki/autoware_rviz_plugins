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

/**
 * @file predicted_objects_display_v2.hpp
 * @brief RViz display plugin for visualizing predicted objects using low-level Ogre rendering
 */

#ifndef AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECTS__PREDICTED_OBJECTS_DISPLAY_V2_HPP_
#define AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECTS__PREDICTED_OBJECTS_DISPLAY_V2_HPP_

#include "autoware_perception_rviz_plugin/common/text_object.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_rendering/objects/shape.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <OgreSceneNode.h>

#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>

/**
 * @namespace autoware_perception_rviz_plugin::objects
 * @brief Contains RViz display plugins for visualizing perception objects
 */
namespace autoware_perception_rviz_plugin::objects
{

/**
 * @namespace autoware_perception_rviz_plugin::objects::detail
 * @brief Internal helper functions for object visualization
 */
namespace detail
{
/**
 * @brief Convert object classification label to human-readable string
 * @param label Object classification label from ObjectClassification message
 * @return Human-readable string representation of the classification
 */
std::string getClassLabel(uint8_t label);

/**
 * @brief Convert Shape message type to Ogre Shape type
 * @param shape_msg Shape message from predicted object
 * @return Corresponding Ogre shape type for rendering
 */
rviz_rendering::Shape::Type getShapeType(const autoware_perception_msgs::msg::Shape & shape_msg);
}  // namespace detail

/**
 * @struct UUIDHash
 * @brief Hash functor for boost::uuids::uuid to use in unordered containers
 *
 * This functor enables using boost::uuids::uuid as a key in std::unordered_map
 * by converting the UUID to a string and hashing it.
 */
struct UUIDHash
{
  /**
   * @brief Hash function operator for boost::uuids::uuid
   * @param uuid The UUID to hash
   * @return Hash value for the UUID
   */
  std::size_t operator()(const boost::uuids::uuid & uuid) const noexcept
  {
    return std::hash<std::string>{}(boost::uuids::to_string(uuid));
  }
};

/**
 * @struct ObjectVisualization
 * @brief Visualization components for a single predicted object
 *
 * This structure holds all the rendering components needed to visualize
 * a single predicted object in the RViz 3D scene.
 */
struct ObjectVisualization
{
  std::unique_ptr<rviz_rendering::Shape> shape;    ///< 3D shape representation of the object
  std::unique_ptr<common::TextObject> label_text;  ///< Classification label text above object
  Ogre::SceneNode * root_node{nullptr};            ///< Root scene node for all components
};

/**
 * @class PredictedObjectsDisplayV2
 * @brief RViz display plugin for visualizing predicted objects with low-level Ogre rendering
 *
 * This display plugin subscribes to PredictedObjects messages and renders each object
 * as a 3D shape with optional classification labels. It uses direct Ogre scene graph
 * manipulation for efficient rendering and supports per-class color configuration.
 *
 * Features:
 * - Direct Ogre scene node management for optimal performance
 * - Per-class color customization
 * - Optional shape and label visibility controls
 * - UUID-based object tracking for efficient updates
 */
class PredictedObjectsDisplayV2 : public rviz_common::Display
{
  Q_OBJECT

public:
  using PredictedObjects = autoware_perception_msgs::msg::PredictedObjects;
  using UUID = boost::uuids::uuid;

  /**
   * @brief Construct a new PredictedObjectsDisplayV2
   */
  PredictedObjectsDisplayV2();

  /**
   * @brief Destroy the PredictedObjectsDisplayV2 and clean up resources
   */
  ~PredictedObjectsDisplayV2() override;

  // Non-copyable and non-movable
  PredictedObjectsDisplayV2(const PredictedObjectsDisplayV2 &) = delete;
  PredictedObjectsDisplayV2 & operator=(const PredictedObjectsDisplayV2 &) = delete;
  PredictedObjectsDisplayV2(PredictedObjectsDisplayV2 &&) = delete;
  PredictedObjectsDisplayV2 & operator=(PredictedObjectsDisplayV2 &&) = delete;

protected:
  /**
   * @brief Initialize the display after construction
   *
   * Sets up properties, initializes ROS subscription, and prepares scene nodes.
   */
  void onInitialize() override;

  /**
   * @brief Reset the display to initial state
   *
   * Clears all object visualizations and resets internal state.
   */
  void reset() override;

  /**
   * @brief Update visualization per frame
   * @param wall_dt Wall clock time elapsed since last update (seconds)
   * @param ros_dt ROS time elapsed since last update (seconds)
   */
  void update(float wall_dt, float ros_dt) override;

  /**
   * @brief Called when display is enabled
   *
   * Subscribes to the PredictedObjects topic.
   */
  void onEnable() override;

  /**
   * @brief Called when display is disabled
   *
   * Unsubscribes from the topic and hides all visualizations.
   */
  void onDisable() override;

private Q_SLOTS:
  /**
   * @brief Update ROS topic subscription when topic property changes
   *
   * Called automatically when the user modifies the topic property in RViz.
   */
  void updateTopic();

  /**
   * @brief Callback for incoming PredictedObjects messages
   * @param msg Shared pointer to the received PredictedObjects message
   */
  void onObjectsReceived(const PredictedObjects::ConstSharedPtr msg);

  /**
   * @brief Update all object visualizations from latest message
   * @param msg Shared pointer to the PredictedObjects message to visualize
   */
  void updateObjects(const PredictedObjects::ConstSharedPtr msg);

  /**
   * @brief Create new visualization for an object
   * @param object The predicted object to visualize
   * @param uuid Unique identifier for the object
   */
  void createObjectVisualization(
    const autoware_perception_msgs::msg::PredictedObject & object, const UUID & uuid);

  /**
   * @brief Update existing object visualization with new data
   * @param vis Reference to the existing ObjectVisualization to update
   * @param object The predicted object with updated data
   */
  void updateObjectVisualization(
    ObjectVisualization & vis, const autoware_perception_msgs::msg::PredictedObject & object);

  /**
   * @brief Remove visualizations for objects no longer in the scene
   * @param current_uuids Set of UUIDs for objects currently in the scene
   */
  void removeStaleObjects(const std::set<UUID> & current_uuids);

  /**
   * @brief Convert ROS UUID message to boost::uuids::uuid
   * @param uuid_msg ROS UUID message to convert
   * @return boost::uuids::uuid representation
   */
  static UUID toBoostUuid(const unique_identifier_msgs::msg::UUID & uuid_msg);

  /**
   * @brief Get color for object based on classification label
   * @param label Object classification label
   * @return Ogre color value for the specified classification
   */
  Ogre::ColourValue getClassColor(uint8_t label) const;

private:  // NOLINT
  // Subscriptions
  /// @brief ROS subscription for PredictedObjects messages
  rclcpp::Subscription<PredictedObjects>::SharedPtr objects_sub_;

  // Properties
  /// @brief ROS topic property for selecting the PredictedObjects topic
  std::unique_ptr<rviz_common::properties::RosTopicProperty> topic_property_;
  /// @brief Property to toggle shape visibility
  std::unique_ptr<rviz_common::properties::BoolProperty> show_shape_property_;
  /// @brief Property to toggle label text visibility
  std::unique_ptr<rviz_common::properties::BoolProperty> show_label_property_;
  /// @brief Property to control shape line width
  std::unique_ptr<rviz_common::properties::FloatProperty> line_width_property_;

  // Per-class color properties
  /// @brief Color property for unknown classification
  std::unique_ptr<rviz_common::properties::ColorProperty> color_unknown_property_;
  /// @brief Color property for car classification
  std::unique_ptr<rviz_common::properties::ColorProperty> color_car_property_;
  /// @brief Color property for truck classification
  std::unique_ptr<rviz_common::properties::ColorProperty> color_truck_property_;
  /// @brief Color property for bus classification
  std::unique_ptr<rviz_common::properties::ColorProperty> color_bus_property_;
  /// @brief Color property for trailer classification
  std::unique_ptr<rviz_common::properties::ColorProperty> color_trailer_property_;
  /// @brief Color property for motorcycle classification
  std::unique_ptr<rviz_common::properties::ColorProperty> color_motorcycle_property_;
  /// @brief Color property for bicycle classification
  std::unique_ptr<rviz_common::properties::ColorProperty> color_bicycle_property_;
  /// @brief Color property for pedestrian classification
  std::unique_ptr<rviz_common::properties::ColorProperty> color_pedestrian_property_;

  // Object tracking
  /// @brief Map of UUID to visualization components for each tracked object
  std::unordered_map<UUID, ObjectVisualization, UUIDHash> object_displays_;

  // Synchronization
  /// @brief Mutex for thread-safe access to shared data
  std::mutex mutex_;
  /// @brief Most recently received PredictedObjects message
  PredictedObjects::ConstSharedPtr latest_msg_;
};

}  // namespace autoware_perception_rviz_plugin::objects

#endif  // AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECTS__PREDICTED_OBJECTS_DISPLAY_V2_HPP_
