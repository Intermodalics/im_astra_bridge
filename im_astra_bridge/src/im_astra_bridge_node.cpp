#include <Astra/Astra.h>
#include <AstraUL/AstraUL.h>
#include <chrono>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <string>

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PclPointCloud;
typedef sensor_msgs::PointCloud2::Ptr RosPointCloud;

/**
 * Class for maintaining the depth, ir and color streams. It iplements the
 * listener-subscriber method as it is used for all example clodes in the astra
 * sdk.
 */
class PointFrameListener : public astra::FrameReadyListener {
  public:
    // A changing pointer to const 3d point clouds.
    const astra::Vector3f* pointData;

    // A changing pointer to const ir color images.
    const astra::RGBPixel* irRGB;

    // Width and height of the 3d point cloud.
    size_t width;
    size_t height;
    size_t width_ir;
    size_t height_ir;

    PointFrameListener() :
      pointData(), 
      width(0), 
      height(0),
      width_ir(0),
      height_ir(0)
      {}

    virtual void on_frame_ready(astra::StreamReader& reader,
      astra::Frame& frame) override {

      // Read a new 3d point frame.
      astra::PointFrame pointFrame = frame.get<astra::PointFrame>();
      astra::InfraredFrameRGB irFrame = frame.get<astra::InfraredFrameRGB>();

      // Retrieve date from 3d point frame.
      if (pointFrame.is_valid()) {
        
        // Catch latest 3d point frame.
        astra::Frame frame = reader.get_latest_frame();

        // Resolution of the astra 3d point frame.
        this->width = pointFrame.resolutionX();
        this->height = pointFrame.resolutionY();

        // Access the 3d point cloud.
        this->pointData = pointFrame.data();

      } else {
        ROS_ERROR_ONCE("Point frame is not valid.");
      }
      
      // Retrieve data from ir rgb color frame.
      if (irFrame.is_valid()) {

        // Catch the latest ir color frame.
        astra::PointFrame pointFrame = frame.get<astra::PointFrame>();

        // Resolution of the astra color frame.
        this->width_ir = irFrame.resolutionX();
        this->height_ir = irFrame.resolutionY();

        // Acces ir color image.
        this->irRGB = irFrame.data();

      } else {
        ROS_ERROR_ONCE("The ir rgb color frame is not valid.");
      }
    }

    /**
     * Configuration of for depth stream. 
     */
    astra::DepthStream configure_depth(astra::StreamReader& reader)
    {
      auto depthStream = reader.stream<astra::DepthStream>();

      // Astra: "We don't have to set the mode to start the stream, 
      // but if you want to here is how:"
      astra::ImageStreamMode depthMode;

      depthMode.set_width(640);
      depthMode.set_height(480);
      depthMode.set_pixelFormat(
        astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
      depthMode.set_fps(30);

      depthStream.set_mode(depthMode);

      return depthStream;
    }

    /**
     * Configuration of for color stream. 
     */
    astra::ColorStream configure_color(astra::StreamReader& reader) {
      auto colorStream = reader.stream<astra::ColorStream>();

      astra::ImageStreamMode colorMode;

      colorMode.set_width(640);
      colorMode.set_height(480);
      colorMode.set_pixelFormat(
        astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
      colorMode.set_fps(30);

      colorStream.set_mode(colorMode);

      return colorStream;
    }

    /**
     * Configuration of the infrared stream
     */
    astra::InfraredStream configure_ir(astra::StreamReader& reader, 
      bool useRGB) {

      auto irStream = reader.stream<astra::InfraredStream>();

      astra::ImageStreamMode irMode;

      irMode.set_width(640);
      irMode.set_height(480);
      if (useRGB)
      {
          irMode.set_pixelFormat(astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
      }
      else
      {
          irMode.set_pixelFormat(astra_pixel_formats::ASTRA_PIXEL_FORMAT_GRAY16);
      }

      irMode.set_fps(30);

      irStream.set_mode(irMode);

      return irStream;
    }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "im_astra_bridge_node");
  ros::NodeHandle n("~");

  // Init astra.
  astra::Astra::initialize();

  // Create astra stream reader.
  astra::StreamSet streamset;
  astra::StreamReader reader = streamset.create_reader();
  
  // Start the 3d point stream.
  reader.stream<astra::PointStream>().start();
  reader.stream<astra::DepthStream>().start();
  
  // Add 3d point cloud listener to point stream reader.
  PointFrameListener listener;
  
  //// Start the depth stream.
  //auto depth_stream = listener.configure_depth(reader);
  //depth_stream.start();

  //// Start the color stream.
  auto color_stream = listener.configure_color(reader);
  color_stream.start();

  //// Start the ir stream.
  //auto ir_stream = listener.configure_ir(reader, true);
  //ir_stream.start();
 
  // Add the depth and color stream listener.
  reader.addListener(listener);

  // Create the publishers.
  ros::Publisher cloud_pub = 
    n.advertise<sensor_msgs::PointCloud2>("depth/points", 1);

  // Set the loop rate.
  ros::Rate loop_rate(30);

  ros::Duration(1.0).sleep();
  ros::spinOnce();

  while (ros::ok()){
    // Update 3d point frame listener.
    astra_temp_update();

    // Create new point cloud.
    PclPointCloud cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->header.frame_id = "base_link";
    const size_t width = listener.width;
    const size_t height = listener.height;
    const size_t width_ir = listener.width_ir;
    const size_t height_ir = listener.height_ir;
    const astra::Vector3f* pointData = listener.pointData;
    const astra::RGBPixel* irRGB = listener.irRGB;
    cloud->width = width;
    cloud->height = height;
    //cloud->is_dense = false;
    cloud->points.resize(width * height);
    
    // Check the usb camera connection.
    if (width == 0 || height == 0) {
      ROS_ERROR_ONCE("3d point cloud size is zero.\n");
    }
    
    // Check correspondence of color image and 3d point cloud.
    if (width != width_ir || height != height_ir) {
      if (width_ir == 0 || height_ir == 0) {
        ROS_ERROR_ONCE("Color image size is zero.\n");
      } else {
        ROS_ERROR_ONCE("Color image size and 3d point cloud size differs.\n");
      }
    }

    // Fill point cloud with 3d points from orbbec camera.
    for (unsigned y = 0; y < height; ++y) {
      for (unsigned x = 0; x < width; ++x, ++pointData) {
        cloud->points[(y * width + x)].x = (*pointData).x / 1000.0;
        cloud->points[(y * width + x)].y = (*pointData).y / 1000.0;
        cloud->points[(y * width + x)].z = (*pointData).z / 1000.0;
        cloud->points[(y * width + x)].r = (*irRGB).r;
        cloud->points[(y * width + x)].g = (*irRGB).g;
        cloud->points[(y * width + x)].b = (*irRGB).b;
      }
    }

    // Convert to sensormsg::Pointcloud2 type.
    RosPointCloud final_cloud (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud, *final_cloud);
    final_cloud->header.frame_id = "base_link";
    final_cloud->header.stamp = ros::Time::now();
    cloud_pub.publish(final_cloud);
    
  }

  // Close astra point stream.
  astra::Astra::terminate();

  return 0;
}
