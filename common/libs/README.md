# common_lib
　The common library for other Lidar Perception packages.

## Functions list

### Tools
+ Self-defined types, _[more](./include/common/types)_.
    + PCL convenient structures, _[more](./include/common/types/type.h#L10)_.
    + Parameters (`yaml`) structures, _[more](./include/common/types/type.h#L74)_.
    + Constructed `Object` structure, _[more](./include/common/types/object.hpp)_.
+ Compare, display, and crop functions, _[more](./include/common/common.hpp)_.
+ Timer helper class, run-time statistics, _[more](./include/common/time.hpp)_.
+ Point Cloud, Point, Object, etc, 3D Transform helping functions, _[more](./include/common/transform.hpp)_.
+ Compute 3D geometry information, _[more](./include/common/geometry.hpp)_.
+ ROS Parameters (in `yaml`) loading functions, _[more](./include/common/parameter.hpp)_.
+ ROS rviz processing results publisher, _[more](./include/common/publisher.hpp)_.
+ 2D & 3D bounding box process functions, _[more](./include/common/bounding_box.hpp)_.
+ Colors defined for rviz visualization, _[more](./include/common/color.hpp)_.

### Algorithms
+ A high-performance 2D `surface::ConvexHull` for Point Cloud, _[more](./include/common/algos/convex_hullxy.hpp)_.
+ Graph Algorithms, _[more](./include/common/algos/graph.hpp)_.
+ Google's Hungarian Optimizer, _[more](./include/common/algos/hungarian_bigraph_matcher.hpp)_.
