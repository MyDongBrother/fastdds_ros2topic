mkdir -p generated/geometry_msgs/msg
fastddsgen -cs -replace -I idl -d generated/geometry_msgs/msg idl/geometry_msgs/msg/PolygonStamped.idl