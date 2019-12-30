#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>

#include "mloam_pcl/point_with_cov.hpp"
#include "mloam_pcl/voxel_grid_covariance_mloam.h"
#include "mloam_pcl/voxel_grid_covariance_mloam_impl.hpp"

#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>

PCL_INSTANTIATE(VoxelGridCovarianceMLOAM, (pcl::PointXYZI)(pcl::PointIWithCov));
// template class PCL_EXPORTS pcl::VoxelGridCovarianceMLOAM<pcl::PointXYZ>;
// template class PCL_EXPORTS pcl::VoxelGridCovarianceMLOAM<pcl::PointIWithCov>;
