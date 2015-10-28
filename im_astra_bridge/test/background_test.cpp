#include <gtest/gtest.h>

#include "../src/im_localiser-component.hpp"

#include <ros/ros.h>
#include <rtt/base/PortInterface.hpp>
#include <rtt/internal/GlobalEngine.hpp>
#include <rtt/os/main.h>
#include <rtt/plugin/PluginLoader.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/OperationCaller.hpp>

static const std::string kFolderPlaneBackground = "./background-plane";
static const std::string kFolderPointBackground = "./background-point";
static const std::string kFolderSphereBackground = "./background-sphere";

class BackgroundTest : public::testing::Test {
  public:
    BackgroundTest()
        : localiser_tc_("localiser"),
          bgcloud_in_port_(new RTT::InputPort<sensor_msgs::PointCloud2>("bgplane_cloud_in_port")) {}
    ~BackgroundTest() {
        delete bgcloud_in_port_;
    }
    void SetUp() {
        ASSERT_TRUE(localiser_tc_.ready());
        bgplane_cloud_out_port_  = static_cast<RTT::OutputPort<sensor_msgs::PointCloud2>* >(localiser_tc_.getPort("bgplane_cloud_port"));
        bgsphere_cloud_out_port_ = static_cast<RTT::OutputPort<sensor_msgs::PointCloud2>* >(localiser_tc_.getPort("bgsphere_cloud_port"));
        bgcloud_in_port_->connectTo(bgplane_cloud_out_port_);
        bgcloud_in_port_->connectTo(bgsphere_cloud_out_port_);

        init_plane_based_background_filter_  = localiser_tc_.getOperation("initPlaneBasedBackgroundFilter");
        init_point_based_background_filter_  = localiser_tc_.getOperation("initPointBasedBackgroundFilter");
        init_sphere_based_background_filter_ = localiser_tc_.getOperation("initSphereBasedBackgroundFilter");

        init_plane_based_background_filter_.setCaller(RTT::internal::GlobalEngine::Instance());
        init_point_based_background_filter_.setCaller(RTT::internal::GlobalEngine::Instance());
        init_sphere_based_background_filter_.setCaller(RTT::internal::GlobalEngine::Instance());

        ASSERT_TRUE(init_plane_based_background_filter_.ready())  << "Failed to get operation caller for init_plane_based_background_filter_";
        ASSERT_TRUE(init_point_based_background_filter_.ready())  << "Failed to get operation caller for init_point_based_background_filter_";
        ASSERT_TRUE(init_sphere_based_background_filter_.ready()) << "Failed to get operation caller for init_sphere_based_background_filter_";
    }
    void TearDown() {}
  protected:
    Im_localiser localiser_tc_;
    RTT::OperationCaller<bool(std::string)> init_plane_based_background_filter_;
    RTT::OperationCaller<bool(std::string)> init_point_based_background_filter_;
    RTT::OperationCaller<bool(std::string)> init_sphere_based_background_filter_;
    RTT::OutputPort<sensor_msgs::PointCloud2>* bgplane_cloud_out_port_;
    RTT::OutputPort<sensor_msgs::PointCloud2>* bgsphere_cloud_out_port_;
    RTT::InputPort<sensor_msgs::PointCloud2>*  bgcloud_in_port_;
};

TEST_F(BackgroundTest, InitPlaneBasedBackgroundFilter) {
    ASSERT_TRUE(init_plane_based_background_filter_(kFolderPlaneBackground));
    sensor_msgs::PointCloud2 plane_based_filtered_bgcloud;
    ASSERT_NE(RTT::NoData, bgcloud_in_port_->read(plane_based_filtered_bgcloud));
}

TEST_F(BackgroundTest, initPointBasedBackgroundFilter) {
    ASSERT_TRUE(init_point_based_background_filter_(kFolderPointBackground));
    // No cloud is witten to a port.
}

TEST_F(BackgroundTest, initSphereBasedBackgroundFilter) {
    ASSERT_TRUE(init_sphere_based_background_filter_(kFolderSphereBackground));
    sensor_msgs::PointCloud2 sphere_based_filtered_bgcloud;
    ASSERT_NE(RTT::NoData, bgcloud_in_port_->read(sphere_based_filtered_bgcloud));
}

int ORO_main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "background_test_node");
    return RUN_ALL_TESTS();
}
