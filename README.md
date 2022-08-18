# obstacle_server_cpp
Obstacle server is a node fusing data about obstacles from various sources. The idea is to provide reliable and aggregated data to planners.

This implementation is split into serveral threads using timers:

- serverPublish() - periodically sending obstacle data to other nodes,
- transformListener() - periodically listening to /tf for transform map->laser.
