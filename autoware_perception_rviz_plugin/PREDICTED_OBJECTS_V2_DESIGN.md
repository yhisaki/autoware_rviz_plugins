# PredictedObjectsDisplayV2 Design Document

## Overview

This document outlines the design for migrating from the marker-based `PredictedObjectsDisplay` to a low-level Ogre-based `PredictedObjectsDisplayV2` implementation, following the pattern established in `TrafficLightDisplay`.

## Current Implementation Analysis

### Old Implementation (`object_detection/predicted_objects_display.cpp`)

**Architecture:**

- Inherits from `ObjectPolygonDisplayBase<PredictedObjects>`
- Uses `visualization_msgs::msg::Marker` and `MarkerArray` for rendering
- Employs thread pool pattern with worker threads for marker generation
- Uses UUID-based object tracking with ID mapping system
- Delegates all rendering to RViz's marker system

**Key Components:**

1. **Thread Management**: Worker thread pool for async marker creation
2. **UUID Tracking**: Maps object UUIDs to marker IDs for persistent visualization
3. **Marker Generation**: Creates 10+ different marker types per object:
   - Shape marker (3D/2D polygon)
   - Label text marker
   - UUID text marker
   - Pose covariance marker
   - Yaw covariance marker
   - Velocity text marker
   - Acceleration text marker
   - Twist (velocity arrow) marker
   - Twist covariance marker
   - Yaw rate marker
   - Yaw rate covariance marker
   - Predicted path markers (multiple per object)
   - Path confidence text markers

**Performance Characteristics:**

- High memory overhead: Each marker is a separate ROS message
- CPU overhead: Marker creation/serialization on every update
- Thread synchronization overhead
- Memory fragmentation from many small allocations

### Reference Implementation (`traffic_light/traffic_light_display.cpp`)

**Architecture:**

- Inherits directly from `rviz_common::Display`
- Uses low-level Ogre primitives (`Shape`, `TextObject`)
- Direct scene node management
- Efficient resource reuse through object pooling
- No marker serialization overhead

**Key Patterns:**

1. **Direct Ogre Access**: Creates `rviz_rendering::Shape` and custom `TextObject` directly
2. **Object Pooling**: Reuses visualization objects via `std::unordered_map<Id, Object>`
3. **Minimal Updates**: Only updates changed properties, not entire objects
4. **RAII Resource Management**: Automatic cleanup via smart pointers

## Proposed V2 Architecture

### Core Design Principles

1. **Low-Level Rendering**: Direct Ogre primitive usage (no markers)
2. **Object Pooling**: Maintain persistent visualization objects
3. **Efficient Updates**: Update only changed properties
4. **Modular Components**: Reusable visualization components
5. **Memory Efficiency**: Minimize allocations per frame

### Class Structure

```cpp
class PredictedObjectsDisplayV2 : public rviz_common::Display
{
  // Core subscription
  rclcpp::Subscription<PredictedObjects>::SharedPtr objects_sub_;

  // Object tracking
  std::unordered_map<UUID, ObjectVisualization> object_displays_;

  // Properties (subset of old implementation for MVP)
  std::unique_ptr<RosTopicProperty> topic_property_;
  std::unique_ptr<BoolProperty> show_shape_property_;
  std::unique_ptr<BoolProperty> show_label_property_;
  std::unique_ptr<BoolProperty> show_velocity_property_;
  std::unique_ptr<BoolProperty> show_paths_property_;
  std::unique_ptr<FloatProperty> line_width_property_;

  // Per-class color properties
  std::unordered_map<uint8_t, ColorProperty> class_colors_;
};
```

### Visualization Components

#### 1. ObjectVisualization Class

Encapsulates all visualization elements for a single tracked object:

```cpp
struct ObjectVisualization
{
  // Shape rendering
  std::unique_ptr<rviz_rendering::Shape> shape;           // Box/cylinder/polygon

  // Text rendering
  std::unique_ptr<common::TextObject> label_text;         // Class label
  std::unique_ptr<common::TextObject> velocity_text;      // Velocity info
  std::unique_ptr<common::TextObject> uuid_text;          // Object ID

  // Arrows/lines
  std::unique_ptr<rviz_rendering::Arrow> velocity_arrow;  // Velocity vector
  std::vector<PathVisualization> predicted_paths;         // Path predictions

  // Covariance visualization
  std::unique_ptr<rviz_rendering::Shape> pose_covariance; // Ellipse
  std::unique_ptr<rviz_rendering::Shape> twist_covariance;

  Ogre::SceneNode* root_node;                            // Parent node for all elements
};
```

#### 2. PathVisualization Class

Handles predicted path rendering:

```cpp
struct PathVisualization
{
  std::unique_ptr<rviz_rendering::BillboardLine> path_line;
  std::vector<std::unique_ptr<rviz_rendering::Shape>> path_poses; // Optional pose markers
  std::unique_ptr<common::TextObject> confidence_text;
};
```

### Implementation Phases

#### Phase 1: Basic Shape & Label (MVP)

**Goal**: Replace marker-based rendering with direct Ogre for core features

**Scope**:

- Subscribe to `PredictedObjects` topic
- Render object shapes (box/cylinder/polygon)
- Display class labels
- UUID-based object tracking
- Basic color mapping by class

**Files to Create/Modify**:

- `predicted_objects_display_v2.hpp` (expand skeleton)
- `predicted_objects_display_v2.cpp` (implement core logic)
- Reuse existing `common/text_object.hpp`

**Estimated LOC**: ~400 lines

#### Phase 2: Velocity & Dynamics

**Scope**:

- Velocity text display
- Velocity arrow visualization
- Acceleration text (if enabled)
- Twist markers

**Estimated LOC**: +200 lines

#### Phase 3: Predicted Paths

**Scope**:

- Multiple predicted path rendering
- Path confidence display
- Color coding by path probability

**Estimated LOC**: +250 lines

#### Phase 4: Covariance Visualization

**Scope**:

- Pose covariance ellipses
- Yaw covariance arcs
- Twist covariance visualization

**Estimated LOC**: +200 lines

#### Phase 5: Advanced Features & Optimization

**Scope**:

- LOD (Level of Detail) system
- Frustum culling optimization
- Property refinement
- Performance profiling

**Estimated LOC**: +150 lines

### Key Differences from Old Implementation

| Aspect              | Old (Marker-based)      | New (Ogre-based)       |
| ------------------- | ----------------------- | ---------------------- |
| **Rendering**       | Marker messages         | Direct Ogre primitives |
| **Threading**       | Worker thread pool      | Single-threaded update |
| **Memory**          | Per-frame allocation    | Object pooling         |
| **Update Cost**     | Full marker rebuild     | Incremental updates    |
| **Serialization**   | ROS msg serialization   | Zero serialization     |
| **Code Complexity** | High (threads, markers) | Medium (Ogre API)      |

### Performance Expectations

**Expected Improvements**:

- **Memory**: 60-70% reduction (no marker messages)
- **CPU**: 40-50% reduction (no serialization, fewer allocations)
- **Frame Time**: 30-40% faster updates for 100+ objects
- **Latency**: Removed thread synchronization overhead

**Trade-offs**:

- More direct Ogre API knowledge required
- Less abstraction than marker system
- Manual resource management (mitigated by RAII)

### Migration Strategy

1. **Parallel Development**: Keep old implementation during V2 development
2. **Feature Parity Testing**: Ensure visual equivalence for each phase
3. **Performance Benchmarking**: Measure improvements at each phase
4. **Gradual Deprecation**: Mark old implementation deprecated after V2 stabilization
5. **Documentation**: Provide migration guide for users

### Dependencies

**Existing Components to Reuse**:

- `common::TextObject` (already implemented)
- Color management from `ObjectPolygonDisplayBase`
- Classification label mapping from `object_polygon_detail.hpp`

**New Components Needed**:

- `PathVisualization` helper class
- Covariance rendering utilities
- UUID hashing for object tracking

### Testing Plan

1. **Unit Tests**: Test object lifecycle, UUID tracking
2. **Visual Tests**: Compare rendering with old implementation
3. **Performance Tests**: Measure frame time, memory usage with varying object counts
4. **Stress Tests**: Handle 200+ objects with multiple paths

### Success Metrics

- [ ] Visual parity with old implementation for all features
- [ ] 40%+ reduction in CPU usage for 100 object scenario
- [ ] 50%+ reduction in memory usage
- [ ] No visual artifacts or rendering bugs
- [ ] Smooth 60 FPS rendering with 150+ objects
- [ ] All original properties supported

## Implementation Notes

### UUID Tracking System

```cpp
using UUID = boost::uuids::uuid;
std::unordered_map<UUID, ObjectVisualization, boost::hash<UUID>> object_map_;

void updateObjects(const PredictedObjects::ConstSharedPtr& msg) {
  std::set<UUID> current_uuids;

  for (const auto& obj : msg->objects) {
    UUID uuid = to_boost_uuid(obj.object_id);
    current_uuids.insert(uuid);

    if (object_map_.find(uuid) == object_map_.end()) {
      // Create new visualization
      object_map_[uuid] = createObjectVisualization(obj);
    } else {
      // Update existing visualization
      updateObjectVisualization(object_map_[uuid], obj);
    }
  }

  // Remove stale objects
  removeStaleObjects(current_uuids);
}
```

### Shape Rendering Pattern

```cpp
void updateObjectShape(ObjectVisualization& vis, const Shape& shape_msg,
                       const Pose& pose, uint8_t classification) {
  if (!vis.shape) {
    vis.shape = std::make_unique<rviz_rendering::Shape>(
      getShapeType(shape_msg), scene_manager_, vis.root_node);
  }

  vis.shape->setPosition(toOgre(pose.position));
  vis.shape->setOrientation(toOgre(pose.orientation));
  vis.shape->setScale(toOgre(shape_msg.dimensions));
  vis.shape->setColor(getClassColor(classification));
}
```

## Conclusion

This V2 implementation will provide significant performance improvements while maintaining feature parity with the marker-based approach. The phased implementation allows for incremental development and testing, reducing risk while delivering value at each stage.
