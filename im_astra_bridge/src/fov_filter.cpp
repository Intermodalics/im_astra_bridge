#include "im_astra_bridge/fov_filter.h"

namespace fov_filter {

double distancePointToPlane(const KDL::Vector& point_on_plane,
                            const KDL::Vector& plane_normal,
                            const pcl::PointXYZRGB& point){
    KDL::Vector n = plane_normal / plane_normal.Norm();
    return n.x() * (point_on_plane.x() - point.x) +
           n.y() * (point_on_plane.y() - point.y) +
           n.z() * (point_on_plane.z() - point.z);
}


void removePointsOutsideFieldOfView(const field_of_view& fov,
                                 const double& margin,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    // FOV Plane Normals (direction: pointing away from the field of view).
    KDL::Vector n_xz_neg(cos(fov.angle_xz / 2.0), 0, -sin(fov.angle_xz / 2.0));
    KDL::Vector n_xz_pos(-cos(fov.angle_xz / 2.0), 0, -sin(fov.angle_xz / 2.0));
    KDL::Vector n_yz_neg(0, -cos(fov.angle_yz / 2.0), -sin(fov.angle_yz / 2.0));
    KDL::Vector n_yz_pos(0, cos(fov.angle_yz / 2.0), -sin(fov.angle_yz / 2.0));
    KDL::Vector n_min_range(0.0, 0.0, -1.0);
    KDL::Vector n_max_range(0.0, 0.0, 1.0);

    // FOV Plane Points.
    KDL::Vector p_origin(0.0, 0.0, 0.0);
    KDL::Vector p_min_range(0.0, 0.0, fov.min_depth_range);
    KDL::Vector p_max_range(0.0, 0.0, fov.max_depth_range);

    for (unsigned int j = 0; j < cloud->points.size(); ++j) {
      // Check if the point respects the opening angle limitations.
      if (distancePointToPlane(p_origin, n_xz_neg, cloud->points[j]) > -margin
       || distancePointToPlane(p_origin, n_xz_pos, cloud->points[j]) > -margin
       || distancePointToPlane(p_origin, n_yz_neg, cloud->points[j]) > -margin
       || distancePointToPlane(p_origin, n_yz_pos, cloud->points[j]) > -margin)
      {
          cloud->points[j].x = std::numeric_limits<float>::quiet_NaN();
          cloud->points[j].y = std::numeric_limits<float>::quiet_NaN();
          cloud->points[j].z = std::numeric_limits<float>::quiet_NaN();
      }
      // Check if point respects the depth range limitations.
      if (distancePointToPlane(p_min_range, n_min_range,
              cloud->points[j]) > -margin
      || distancePointToPlane(p_min_range, n_min_range,
              cloud->points[j]) > -margin )
      {
        cloud->points[j].x = std::numeric_limits<float>::quiet_NaN();
        cloud->points[j].y = std::numeric_limits<float>::quiet_NaN();
        cloud->points[j].z = std::numeric_limits<float>::quiet_NaN();
      }
    }
    return;
}

} // namespace fov_filter

