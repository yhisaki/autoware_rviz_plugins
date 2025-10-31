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

#include "autoware_perception_rviz_plugin/objects/predicted_objects_display_v2.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>

namespace autoware_perception_rviz_plugin::objects
{

using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;

namespace detail
{

std::string getClassLabel(uint8_t label)
{
  switch (label) {
    case ObjectClassification::CAR:
      return "CAR";
    case ObjectClassification::TRUCK:
      return "TRUCK";
    case ObjectClassification::BUS:
      return "BUS";
    case ObjectClassification::TRAILER:
      return "TRAILER";
    case ObjectClassification::MOTORCYCLE:
      return "MOTORCYCLE";
    case ObjectClassification::BICYCLE:
      return "BICYCLE";
    case ObjectClassification::PEDESTRIAN:
      return "PEDESTRIAN";
    default:
      return "UNKNOWN";
  }
}

rviz_rendering::Shape::Type getShapeType(const Shape & shape_msg)
{
  switch (shape_msg.type) {
    case Shape::BOUNDING_BOX:
      return rviz_rendering::Shape::Cube;
    case Shape::CYLINDER:
      return rviz_rendering::Shape::Cylinder;
    case Shape::POLYGON:
      // TODO(Phase 4): Implement proper polygon rendering
      return rviz_rendering::Shape::Cube;
    default:
      return rviz_rendering::Shape::Cube;
  }
}

}  // namespace detail

PredictedObjectsDisplayV2::PredictedObjectsDisplayV2() = default;

PredictedObjectsDisplayV2::~PredictedObjectsDisplayV2() = default;

void PredictedObjectsDisplayV2::onInitialize()
{
  rviz_common::Display::onInitialize();

  auto rviz_ros_node = context_->getRosNodeAbstraction();

  // Topic property
  topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>(
    "Topic", QString("/perception/object_recognition/objects"),
    QString("autoware_perception_msgs/msg/PredictedObjects"),
    QString("Topic for predicted objects data"), this, SLOT(updateTopic()));
  topic_property_->initialize(rviz_ros_node);

  // Display properties
  show_shape_property_ = std::make_unique<rviz_common::properties::BoolProperty>(
    "Show Shape", true, "Show/hide object shapes", this);

  show_label_property_ = std::make_unique<rviz_common::properties::BoolProperty>(
    "Show Label", true, "Show/hide object labels", this);

  line_width_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
    "Line Width", 0.03, "Line width for object boundaries", this);
  line_width_property_->setMin(0.01);

  // Color properties for each class
  color_unknown_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
    "Color: Unknown", QColor(255, 255, 255), "Color for unknown objects", this);

  color_car_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
    "Color: Car", QColor(255, 200, 0), "Color for car objects", this);

  color_truck_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
    "Color: Truck", QColor(255, 150, 0), "Color for truck objects", this);

  color_bus_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
    "Color: Bus", QColor(200, 100, 0), "Color for bus objects", this);

  color_trailer_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
    "Color: Trailer", QColor(150, 100, 50), "Color for trailer objects", this);

  color_motorcycle_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
    "Color: Motorcycle", QColor(0, 200, 255), "Color for motorcycle objects", this);

  color_bicycle_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
    "Color: Bicycle", QColor(0, 150, 255), "Color for bicycle objects", this);

  color_pedestrian_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
    "Color: Pedestrian", QColor(255, 100, 255), "Color for pedestrian objects", this);
}

void PredictedObjectsDisplayV2::updateTopic()
{
  objects_sub_.reset();
  auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
  if (rviz_ros_node) {
    objects_sub_ = rviz_ros_node->get_raw_node()->create_subscription<PredictedObjects>(
      topic_property_->getTopicStd(), rclcpp::QoS(10),
      std::bind(&PredictedObjectsDisplayV2::onObjectsReceived, this, std::placeholders::_1));
  }
}

void PredictedObjectsDisplayV2::onEnable()
{
  rviz_common::Display::onEnable();
  updateTopic();
}

void PredictedObjectsDisplayV2::onDisable()
{
  rviz_common::Display::onDisable();
  objects_sub_.reset();

  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & [uuid, vis] : object_displays_) {
    if (vis.root_node) {
      vis.root_node->setVisible(false);
    }
  }
}

void PredictedObjectsDisplayV2::reset()
{
  rviz_common::Display::reset();

  std::lock_guard<std::mutex> lock(mutex_);
  object_displays_.clear();
  latest_msg_.reset();
}

void PredictedObjectsDisplayV2::onObjectsReceived(const PredictedObjects::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_msg_ = msg;
}

void PredictedObjectsDisplayV2::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  std::lock_guard<std::mutex> lock(mutex_);

  if (!latest_msg_) {
    return;
  }

  updateObjects(latest_msg_);
}

void PredictedObjectsDisplayV2::updateObjects(const PredictedObjects::ConstSharedPtr msg)
{
  // Get transform
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Transform",
      QString::fromStdString(
        "Error transforming from frame '" + msg->header.frame_id + "' to frame '" +
        qPrintable(fixed_frame_) + "'"));
    return;
  }
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  // Track current UUIDs
  std::set<UUID> current_uuids;

  for (const auto & object : msg->objects) {
    const UUID uuid = toBoostUuid(object.object_id);
    current_uuids.insert(uuid);

    if (object_displays_.find(uuid) == object_displays_.end()) {
      createObjectVisualization(object, uuid);
    } else {
      updateObjectVisualization(object_displays_[uuid], object);
    }
  }

  removeStaleObjects(current_uuids);
}

void PredictedObjectsDisplayV2::createObjectVisualization(
  const autoware_perception_msgs::msg::PredictedObject & object, const UUID & uuid)
{
  ObjectVisualization vis;

  // Create root scene node
  vis.root_node = scene_node_->createChildSceneNode();

  // Create shape
  if (show_shape_property_->getBool()) {
    const auto shape_type = detail::getShapeType(object.shape);
    vis.shape = std::make_unique<rviz_rendering::Shape>(shape_type, scene_manager_, vis.root_node);

    // Set position and orientation
    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    vis.shape->setPosition(Ogre::Vector3(
      static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
      static_cast<float>(pose.position.z)));
    vis.shape->setOrientation(Ogre::Quaternion(
      static_cast<float>(pose.orientation.w), static_cast<float>(pose.orientation.x),
      static_cast<float>(pose.orientation.y), static_cast<float>(pose.orientation.z)));

    // Set scale
    vis.shape->setScale(Ogre::Vector3(
      static_cast<float>(object.shape.dimensions.x), static_cast<float>(object.shape.dimensions.y),
      static_cast<float>(object.shape.dimensions.z)));

    // Set color based on classification
    const uint8_t label = object.classification.empty() ? ObjectClassification::UNKNOWN
                                                        : object.classification[0].label;
    vis.shape->setColor(getClassColor(label));
  }

  // Create label text
  if (show_label_property_->getBool() && !object.classification.empty()) {
    const uint8_t label = object.classification[0].label;
    const std::string label_str = detail::getClassLabel(label);

    vis.label_text = std::make_unique<common::TextObject>(
      scene_manager_, vis.root_node, label_str, "Liberation Sans", 0.5f);

    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    vis.label_text->setPosition(Ogre::Vector3(
      static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
      static_cast<float>(pose.position.z + object.shape.dimensions.z / 2.0 + 0.3)));

    vis.label_text->setColor(1.0f, 1.0f, 1.0f, 1.0f);
  }

  object_displays_[uuid] = std::move(vis);
}

void PredictedObjectsDisplayV2::updateObjectVisualization(
  ObjectVisualization & vis, const autoware_perception_msgs::msg::PredictedObject & object)
{
  const auto & pose = object.kinematics.initial_pose_with_covariance.pose;

  // Update shape
  if (vis.shape && show_shape_property_->getBool()) {
    vis.shape->setPosition(Ogre::Vector3(
      static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
      static_cast<float>(pose.position.z)));
    vis.shape->setOrientation(Ogre::Quaternion(
      static_cast<float>(pose.orientation.w), static_cast<float>(pose.orientation.x),
      static_cast<float>(pose.orientation.y), static_cast<float>(pose.orientation.z)));

    const uint8_t label = object.classification.empty() ? ObjectClassification::UNKNOWN
                                                        : object.classification[0].label;
    vis.shape->setColor(getClassColor(label));

    vis.shape->getRootNode()->setVisible(true);
  } else if (vis.shape) {
    vis.shape->getRootNode()->setVisible(false);
  }

  // Update label text
  if (vis.label_text && show_label_property_->getBool() && !object.classification.empty()) {
    const uint8_t label = object.classification[0].label;
    const std::string label_str = detail::getClassLabel(label);

    vis.label_text->setCaption(label_str);
    vis.label_text->setPosition(Ogre::Vector3(
      static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
      static_cast<float>(pose.position.z + object.shape.dimensions.z / 2.0 + 0.3)));
    vis.label_text->setVisible(true);
  } else if (vis.label_text) {
    vis.label_text->setVisible(false);
  }
}

void PredictedObjectsDisplayV2::removeStaleObjects(const std::set<UUID> & current_uuids)
{
  auto it = object_displays_.begin();
  while (it != object_displays_.end()) {
    if (current_uuids.find(it->first) == current_uuids.end()) {
      // Remove stale object
      if (it->second.root_node) {
        scene_manager_->destroySceneNode(it->second.root_node);
      }
      it = object_displays_.erase(it);
    } else {
      ++it;
    }
  }
}

PredictedObjectsDisplayV2::UUID PredictedObjectsDisplayV2::toBoostUuid(
  const unique_identifier_msgs::msg::UUID & uuid_msg)
{
  std::stringstream ss;
  for (size_t i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(uuid_msg.uuid[i]);
  }
  boost::uuids::string_generator gen;
  return gen(ss.str());
}

Ogre::ColourValue PredictedObjectsDisplayV2::getClassColor(uint8_t label) const
{
  const rviz_common::properties::ColorProperty * color_property = nullptr;

  switch (label) {
    case ObjectClassification::CAR:
      color_property = color_car_property_.get();
      break;
    case ObjectClassification::TRUCK:
      color_property = color_truck_property_.get();
      break;
    case ObjectClassification::BUS:
      color_property = color_bus_property_.get();
      break;
    case ObjectClassification::TRAILER:
      color_property = color_trailer_property_.get();
      break;
    case ObjectClassification::MOTORCYCLE:
      color_property = color_motorcycle_property_.get();
      break;
    case ObjectClassification::BICYCLE:
      color_property = color_bicycle_property_.get();
      break;
    case ObjectClassification::PEDESTRIAN:
      color_property = color_pedestrian_property_.get();
      break;
    default:
      color_property = color_unknown_property_.get();
      break;
  }

  const QColor & q_color = color_property->getColor();
  return Ogre::ColourValue(
    static_cast<float>(q_color.redF()), static_cast<float>(q_color.greenF()),
    static_cast<float>(q_color.blueF()), static_cast<float>(q_color.alphaF()));
}

}  // namespace autoware_perception_rviz_plugin::objects

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware_perception_rviz_plugin::objects::PredictedObjectsDisplayV2, rviz_common::Display)
