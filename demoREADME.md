# PACKAGE NAME

overall description of package

## Required dependencies
list here

name_of_node_1
=========

### Overview
description of node

### Subscribed topics
* `/topic_name (msgs/Msg_type)` description of topic

### Published topics
* `/topic_name (msg/Msg_type)` description of topic

### Services
* `/service_name (srv/Srv_type)` description of service

### Parameters
* `parameter (type, default:value)` description of parameter

### TF
* required TF transforms `frame1 -> frame2`
* published TF transforms `frame3 -> frame4`

### Custom Messages/Services
 * package/MyMsg.msg

        Header header
        uint32 id
        geometry_msgs/PoseWithCovariance pose
        float64 confidence


name_of_node_2
=========
...



USAGE
=====
* pre-launch requirements
* `roslaunch`
