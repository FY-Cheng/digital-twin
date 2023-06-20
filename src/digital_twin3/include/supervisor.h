# pragma once

#include <signal.h>
#include <stdio.h>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>


#include <webots_ros/get_uint64.h>
#include <webots_ros/field_get_vec2f.h>
#include <webots_ros/field_get_vec3f.h>
#include <webots_ros/field_get_count.h>
#include <webots_ros/field_get_node.h>
#include <webots_ros/field_import_node_from_string.h>
#include <webots_ros/node_get_position.h>
#include <webots_ros/node_get_field.h>
#include <webots_ros/supervisor_get_from_def.h>


class Supervisor
{
public:
    ros::NodeHandle n;
    std::string controllerName;

    Supervisor(const ros::NodeHandle& nh, const std::string& controllerName);
    
    uint64_t supervisor_get_world();
    uint64_t supervisor_get_form_DEF(const std::string& DEF_name);
    uint64_t supervisor_node_get_field(const uint64_t& node, const std::string& field_name);
    int32_t field_import_node_from_string(const uint64_t& field, const int position, const std::string& nodeString);
    geometry_msgs::Vector3 supervisor_field_get_vec2f(const uint64_t& field, const int32_t& index);
    geometry_msgs::Vector3 supervisor_field_get_vec3f(const uint64_t& field, const int32_t& index);
    geometry_msgs::Point supervisor_node_get_position(const uint64_t& node);
    uint64_t supervisor_field_get_node(const uint64_t& field, const int32_t index);
    int32_t supervisor_field_get_count(const uint64_t& field);
};

