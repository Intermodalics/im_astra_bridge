#ifndef FOV_FILTER_HPP
#define FOV_FILTER_HPP

#include <kdl/frames.hpp>
#include <pcl/common/common.h>


namespace fov_filter {

struct field_of_view{

    // opening angle in the XZ plane (degrees).
    double angle_xz;
    // opening angle in the YZ plane (degrees).
    double angle_yz;
    // min depth range (m).
    double min_depth_range;
    // max depth range (m).
    double max_depth_range;
};

double distancePointToPlane(const KDL::Vector& point_on_plane,
                            const KDL::Vector& plane_normal,
                            const pcl::PointXYZRGB& point);

/**
 * The FOV is defined by 6 planes. The first four planes originate from
 * the limited camera opening angle and intersect with the camera origin.
 * The other two planes originate from the limitation on the depth range and
 * are perpendicular to the camera axis. This function sets points that fall
 * outside of the field-of-view to NaN.
 *
 * @param fov
 * @param margin
 * @param cloud
 */
void removePointsOutsideFieldOfView(const field_of_view& fov,
                                  const double& margin,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
};

// namespace fov_filter
#endif // FOV_FILTER_HPP
