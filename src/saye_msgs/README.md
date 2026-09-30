# saye_msgs — Custom ROS 2 Interfaces

`saye_msgs` defines custom ROS 2 message and service interfaces for multi-vehicle coordination, map sharing, and costmap synchronization.

---

## Message Definitions

### `saye_msgs/msg/Map.msg`
Encapsulates combined costmap, occupancy grid, and path trajectory data for inter-robot map distribution:
* `nav2_msgs/CostmapMetaData costmap_meta_data`
* `nav2_msgs/Costmap costmap`
* `nav_msgs/OccupancyGrid occupancy_grid`
* `nav_msgs/MapMetaData map_meta_data`
* `nav_msgs/Path path`
* `std_msgs/Header header`

---

## Service Definitions

### `saye_msgs/srv/ShareMap.srv`
Service request to trigger map sharing across vehicles or nodes:
* **Request:**
  * `string topic_name`
* **Response:**
  * `bool is_completed`
  * `nav_msgs/OccupancyGrid custom_occupany_grid`
  * `string status_message`
