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

#include "autoware_perception_rviz_plugin/common/text_object.hpp"

#include <memory>
#include <string>

namespace autoware_perception_rviz_plugin::common
{

TextObject::TextObject(
  Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node, const std::string & caption,
  const std::string & font_name, float character_height)
: Object(scene_manager), scene_manager_(scene_manager), scene_node_(nullptr)
{
  scene_node_ = parent_node->createChildSceneNode();

  text_ = std::make_unique<rviz_rendering::MovableText>(caption, font_name, character_height);
  text_->setTextAlignment(
    rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);

  scene_node_->attachObject(text_.get());
}

TextObject::~TextObject()
{
  if (scene_node_) {
    if (text_) {
      scene_node_->detachObject(text_.get());
    }
    scene_manager_->destroySceneNode(scene_node_);
  }
}

void TextObject::setCaption(const std::string & caption)
{
  if (text_) {
    text_->setCaption(caption);
  }
}

void TextObject::setCharacterHeight(float height)
{
  if (text_) {
    text_->setCharacterHeight(height);
  }
}

void TextObject::setColor(float r, float g, float b, float a)
{
  if (text_) {
    text_->setColor(Ogre::ColourValue(r, g, b, a));
  }
}

void TextObject::setTextAlignment(
  rviz_rendering::MovableText::HorizontalAlignment horizontal,
  rviz_rendering::MovableText::VerticalAlignment vertical)
{
  if (text_) {
    text_->setTextAlignment(horizontal, vertical);
  }
}

void TextObject::setPosition(const Ogre::Vector3 & position)
{
  if (scene_node_) {
    scene_node_->setPosition(position);
  }
}

void TextObject::setOrientation(const Ogre::Quaternion & orientation)
{
  if (scene_node_) {
    scene_node_->setOrientation(orientation);
  }
}

void TextObject::setScale(const Ogre::Vector3 & scale)
{
  if (scene_node_) {
    scene_node_->setScale(scale);
  }
}

const Ogre::Vector3 & TextObject::getPosition()
{
  static const Ogre::Vector3 default_position = Ogre::Vector3::ZERO;
  return scene_node_ ? scene_node_->getPosition() : default_position;
}

const Ogre::Quaternion & TextObject::getOrientation()
{
  static const Ogre::Quaternion default_orientation = Ogre::Quaternion::IDENTITY;
  return scene_node_ ? scene_node_->getOrientation() : default_orientation;
}

void TextObject::setUserData(const Ogre::Any & data)
{
  if (scene_node_) {
    scene_node_->getUserObjectBindings().setUserAny(data);
  }
}

void TextObject::setVisible(bool visible)
{
  if (scene_node_) {
    scene_node_->setVisible(visible);
  }
}

}  // namespace autoware_perception_rviz_plugin::common
