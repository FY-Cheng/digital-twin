#include "supervisor.h"

Supervisor::Supervisor(const ros::NodeHandle& nh, const std::string& controllerName)
{
    this->n = nh;
    this->controllerName = controllerName;
}

uint64_t Supervisor::supervisor_get_world()
{
    ros::ServiceClient getWorldClient = 
        n.serviceClient<webots_ros::get_uint64>(controllerName + "/supervisor/get_root");
    webots_ros::get_uint64 getWorldSrv;

    if (!getWorldClient.call(getWorldSrv))
    {
        ROS_INFO("Supervisor get world root failed.");
    }

    return getWorldSrv.response.value;
}

uint64_t Supervisor::supervisor_get_form_DEF(const std::string& DEF_name)
{
    ros::ServiceClient getFromDefClient = 
        n.serviceClient<webots_ros::supervisor_get_from_def>(controllerName + "/supervisor/get_from_def");
    webots_ros::supervisor_get_from_def getFromDefSrv;

    getFromDefSrv.request.name = DEF_name;
    if (!getFromDefClient.call(getFromDefSrv))
    {
        ROS_INFO("Supervisor get node from DEF '%s' failed.",DEF_name.c_str());
    }
    getFromDefClient.shutdown();
    return getFromDefSrv.response.node;
}

uint64_t Supervisor::supervisor_node_get_field(const uint64_t& node, const std::string& field_name)
{
    ros::ServiceClient nodeGerFieldClient =
        n.serviceClient<webots_ros::node_get_field>(controllerName + "/supervisor/node/get_field");
    webots_ros::node_get_field nodeGerFieldSrv;

    nodeGerFieldSrv.request.node = node;
    nodeGerFieldSrv.request.fieldName = field_name;
    if (!nodeGerFieldClient.call(nodeGerFieldSrv))
    {
        ROS_INFO("Supervisor get field form node '%s' failed.",field_name.c_str());
    }

    return nodeGerFieldSrv.response.field;
}

int32_t Supervisor::field_import_node_from_string(const uint64_t& field, const int position, const std::string& nodeString)
{
    ros::ServiceClient fieldImportNodeFromStrClient = 
        n.serviceClient<webots_ros::field_import_node_from_string>(controllerName + "/supervisor/field/import_node_from_string");
    webots_ros::field_import_node_from_string fieldImportNodeFromStrSrv;
    
    fieldImportNodeFromStrSrv.request.field = field;
    fieldImportNodeFromStrSrv.request.position = position;
    fieldImportNodeFromStrSrv.request.nodeString = nodeString;

    if (fieldImportNodeFromStrClient.call(fieldImportNodeFromStrSrv) && fieldImportNodeFromStrSrv.response.success)
    {
    }
    else
    {
        ROS_ERROR("Supervisor import node form string '%s' in field failed.",nodeString.c_str());
    }

    return fieldImportNodeFromStrSrv.response.success;
}

geometry_msgs::Vector3 Supervisor::supervisor_field_get_vec2f(const uint64_t& field, const int32_t& index)
{
    ros::ServiceClient fieldGetVector2Client = 
        n.serviceClient<webots_ros::field_get_vec2f>(controllerName + "/supervisor/field/get_vec2f");
    webots_ros::field_get_vec2f fieldGetVector2Srv;

    fieldGetVector2Srv.request.field = field;
    fieldGetVector2Srv.request.index = index;

    if (!fieldGetVector2Client.call(fieldGetVector2Srv))
    {
        ROS_INFO("Supervisor field get vector2 failed.");
    }

    return fieldGetVector2Srv.response.value;
}

geometry_msgs::Vector3 Supervisor::supervisor_field_get_vec3f(const uint64_t& field, const int32_t& index)
{
    ros::ServiceClient fieldGetVector3Client = 
        n.serviceClient<webots_ros::field_get_vec3f>(controllerName + "/supervisor/field/get_vec3f");
    webots_ros::field_get_vec3f fieldGetVector3Srv;

    fieldGetVector3Srv.request.field = field;
    fieldGetVector3Srv.request.index = index;

    if (!fieldGetVector3Client.call(fieldGetVector3Srv))
    {
        ROS_INFO("Supervisor field get vector3 failed.");
    }

    return fieldGetVector3Srv.response.value;
}

geometry_msgs::Point Supervisor::supervisor_node_get_position(const uint64_t& node)
{
    ros::ServiceClient nodeGetPositionClient =
        n.serviceClient<webots_ros::node_get_position>(controllerName + "/supervisor/node/get_position");
    webots_ros::node_get_position nodeGetPositionSrv;

    nodeGetPositionSrv.request.node = node;

    if (!nodeGetPositionClient.call(nodeGetPositionSrv))
    {
        ROS_INFO("Supervisor node get position failed.");
    }

    return nodeGetPositionSrv.response.position;

}

uint64_t Supervisor::supervisor_field_get_node(const uint64_t& field, const int32_t index)
{
    ros::ServiceClient fieldGetNodeClient =
        n.serviceClient<webots_ros::field_get_node>(controllerName + "/supervisor/field/get_node");
    webots_ros::field_get_node fieldGetNodeSrv;

    fieldGetNodeSrv.request.field = field;
    fieldGetNodeSrv.request.index = index;

    if (!fieldGetNodeClient.call(fieldGetNodeSrv))
    {
        ROS_INFO("Supervisor field get node failed.");
    }

    return fieldGetNodeSrv.response.node;
}

int32_t Supervisor::supervisor_field_get_count(const uint64_t& field)
{
    ros::ServiceClient fieldGetCountClient =
        n.serviceClient<webots_ros::field_get_count>(controllerName + "/supervisor/field/get_count");
    webots_ros::field_get_count fieldGetCountSrv;

    fieldGetCountSrv.request.field = field;

    if (!fieldGetCountClient.call(fieldGetCountSrv))
    {
        ROS_INFO("Supervisor field get count failed.");
    }

    return fieldGetCountSrv.response.count;
}