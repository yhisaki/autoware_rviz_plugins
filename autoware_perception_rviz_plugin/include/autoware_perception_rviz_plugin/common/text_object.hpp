// Copyright [Year] [Copyright Holder]
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_PERCEPTION_RVIZ_PLUGIN__COMMON__TEXT_OBJECT_HPP_
#define AUTOWARE_PERCEPTION_RVIZ_PLUGIN__COMMON__TEXT_OBJECT_HPP_

/**
 * @file text_object.hpp
 * @brief Defines the TextObject class for rendering 3D text in RViz
 */

#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/objects/object.hpp>

#include <OgreColourValue.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <memory>
#include <string>

namespace autoware_perception_rviz_plugin::common
{

/**
 * @class TextObject
 * @brief A 3D text rendering object that extends rviz_rendering::Object
 *
 * This class provides a wrapper around rviz_rendering::MovableText to render
 * text in a 3D scene. It manages its own scene node and provides methods to
 * control text appearance, position, and visibility.
 */
class TextObject : public rviz_rendering::Object
{
public:
  /**
   * @brief Construct a new TextObject
   *
   * @param scene_manager Pointer to the Ogre scene manager
   * @param parent_node Pointer to the parent scene node to attach this text to
   * @param caption Initial text caption to display (default: empty string)
   * @param font_name Font name to use for rendering (default: "Liberation Sans")
   * @param character_height Height of characters in scene units (default: 1.0)
   */
  TextObject(
    Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node,
    const std::string & caption = "", const std::string & font_name = "Liberation Sans",
    float character_height = 1.0f);

  /**
   * @brief Destroy the TextObject and clean up resources
   */
  ~TextObject() override;

  // Non-copyable and non-movable
  TextObject(const TextObject &) = delete;
  TextObject & operator=(const TextObject &) = delete;
  TextObject(TextObject &&) = delete;
  TextObject & operator=(TextObject &&) = delete;

  /**
   * @brief Set the text caption
   * @param caption The new text to display
   */
  void setCaption(const std::string & caption);

  /**
   * @brief Set the character height
   * @param height Character height in scene units
   */
  void setCharacterHeight(float height);

  /**
   * @brief Set the text alignment
   * @param horizontal Horizontal alignment (left, center, right)
   * @param vertical Vertical alignment (top, center, bottom)
   */
  void setTextAlignment(
    rviz_rendering::MovableText::HorizontalAlignment horizontal,
    rviz_rendering::MovableText::VerticalAlignment vertical);

  /**
   * @brief Set the visibility of the text
   * @param visible True to show the text, false to hide it
   */
  void setVisible(bool visible);

  // Override methods from rviz_rendering::Object

  /**
   * @brief Set the position of the text in 3D space
   * @param position The position vector
   */
  void setPosition(const Ogre::Vector3 & position) override;

  /**
   * @brief Set the orientation of the text
   * @param orientation The orientation quaternion
   */
  void setOrientation(const Ogre::Quaternion & orientation) override;

  /**
   * @brief Set the scale of the text
   * @param scale The scale vector
   */
  void setScale(const Ogre::Vector3 & scale) override;

  /**
   * @brief Set the color of the text
   * @param r Red component (0.0 to 1.0)
   * @param g Green component (0.0 to 1.0)
   * @param b Blue component (0.0 to 1.0)
   * @param a Alpha component (0.0 to 1.0)
   */
  void setColor(float r, float g, float b, float a) override;

  /**
   * @brief Get the current position of the text
   * @return Reference to the position vector
   */
  const Ogre::Vector3 & getPosition() override;

  /**
   * @brief Get the current orientation of the text
   * @return Reference to the orientation quaternion
   */
  const Ogre::Quaternion & getOrientation() override;

  /**
   * @brief Set user-defined data for this object
   * @param data The user data to associate with this object
   */
  void setUserData(const Ogre::Any & data) override;

  /**
   * @brief Get the underlying scene node
   * @return Pointer to the Ogre scene node
   */
  [[nodiscard]] Ogre::SceneNode * getSceneNode() const { return scene_node_; }

private:
  Ogre::SceneManager * scene_manager_;                 ///< Pointer to the Ogre scene manager
  Ogre::SceneNode * scene_node_;                       ///< Scene node that owns this text object
  std::unique_ptr<rviz_rendering::MovableText> text_;  ///< The underlying movable text object
};

}  // namespace autoware_perception_rviz_plugin::common
#endif  // AUTOWARE_PERCEPTION_RVIZ_PLUGIN__COMMON__TEXT_OBJECT_HPP_
