#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub_filtered_cloud;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud_msg)
{
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*input_cloud_msg, *pcl_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pcl_cloud, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);

    // Filter along x-axis
    /*
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-1.0, 1.0); // Adjust the range as needed
    pass.filter(*cloud_filtered);
    */

    // Filter along y-axis
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2.5, 2.5); // Adjust the range as needed
    pass.filter(*cloud_filtered);

    // Filter along z-axis
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-5.0, 2.5); // Adjust the range as needed
    pass.filter(*cloud_filtered);

    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(*cloud_filtered, output_cloud_msg);
    output_cloud_msg.header = input_cloud_msg->header;

    pub_filtered_cloud.publish(output_cloud_msg);
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "point_cloud_filter");

    // Create a ROS NodeHandle
    ros::NodeHandle nh;

    // Subscribe to the input point cloud topic
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/ouster/points", 1, pointCloudCallback);

    // Advertise the filtered point cloud topic
    pub_filtered_cloud = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1);

    // Spin
    ros::spin();

    return 0;
}

