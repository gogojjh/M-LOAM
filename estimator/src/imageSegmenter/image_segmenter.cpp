#include "image_segmenter.h"

using namespace common;

const float segment_theta = 60.0 / 180.0 * M_PI; // TODO

ImageSegmenter::ImageSegmenter()
{
    std::pair<int8_t, int8_t> neighbor;
    neighbor.first = -1; neighbor.second =  0; neighbor_iterator_.push_back(neighbor);
    neighbor.first =  0; neighbor.second =  1; neighbor_iterator_.push_back(neighbor);
    neighbor.first =  0; neighbor.second = -1; neighbor_iterator_.push_back(neighbor);
    neighbor.first =  1; neighbor.second =  0; neighbor_iterator_.push_back(neighbor);
}

void ImageSegmenter::setScanParam(const int &horizon_scans,
                                    const int &min_cluster_size, const int &min_line_size,
                                    const int &segment_valid_point_num, const int &segment_valid_line_num)
{
    horizon_scans_ = horizon_scans;
    ang_res_x_ = 360.0 / horizon_scans_;
    ang_res_y_ = 2.0; // TODO: adaptive to different scan number
    ang_bottom_ = 15.0+0.1;
    segment_alphax_ = ang_res_x_ / 180.0 * M_PI;
    segment_alphay_ = ang_res_y_ / 180.0 * M_PI;

    min_cluster_size_ = min_cluster_size;
    min_line_size_ = min_line_size;
    segment_valid_point_num_ = segment_valid_point_num;
    segment_valid_line_num_ = segment_valid_line_num;
    // printf("%d, %d, %d, %d\n", min_cluster_size_, min_line_size_, segment_valid_point_num_, segment_valid_line_num_);
}

void ImageSegmenter::setParameter()
{
    cloud_matrix_.reset(new PointCloud());
    cloud_matrix_->points.resize(N_SCANS * horizon_scans_);

    range_mat_ = cv::Mat(N_SCANS, horizon_scans_, CV_32F, cv::Scalar::all(FLT_MAX));
    label_mat_ = cv::Mat(N_SCANS, horizon_scans_, CV_32S, cv::Scalar::all(0));
    label_count_ = 1;

    all_pushed_indx_ = new uint16_t[N_SCANS*horizon_scans_];
    all_pushed_indy_ = new uint16_t[N_SCANS*horizon_scans_];

    queue_indx_ = new uint16_t[N_SCANS*horizon_scans_];
    queue_indy_ = new uint16_t[N_SCANS*horizon_scans_];

    queue_indx_last_negi_ = new int[N_SCANS*horizon_scans_];
    queue_indy_last_negi_ = new int[N_SCANS*horizon_scans_];

    queue_last_dis_ = new float[N_SCANS*horizon_scans_];
}

void ImageSegmenter::projectCloud(const PointCloud &laser_cloud_in)
{
    float vertical_angle, horizon_angle, range;
    size_t row_id, column_id;
    size_t cloud_size = 0;
    Point point;
    for (size_t i = 0; i < laser_cloud_in.points.size(); i++)
    {
        point.x = laser_cloud_in.points[i].x;
        point.y = laser_cloud_in.points[i].y;
        point.z = laser_cloud_in.points[i].z;
        vertical_angle = atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        horizon_angle = atan2(point.x, point.y) * 180 / M_PI;

        row_id = (vertical_angle + ang_bottom_) / ang_res_y_;
        column_id = -round((horizon_angle - 90.0) / ang_res_x_) + horizon_scans_ / 2;
        if (row_id < 0 || row_id >= N_SCANS) continue;
        if (column_id >= horizon_scans_) column_id -= horizon_scans_;
        if (column_id < 0 || column_id >= horizon_scans_) continue;

        range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        range_mat_.at<float>(row_id, column_id) = range;
        // point.intensity = (float)row_id + (float)column_id / 10000.0;
        int index = column_id  + row_id * horizon_scans_;
        cloud_matrix_->points[index] = point;
        cloud_size++;
    }
    // printf("points in range image: %d\n", cloud_size); // simulation: cloud_size == laser_cloud_in.size()
    for (size_t i = 0; i < N_SCANS; i++)
        for (size_t j = 0; j < horizon_scans_; j++)
            if (range_mat_.at<float>(i, j) == FLT_MAX) label_mat_.at<int>(i, j) = -1;
}

void ImageSegmenter::segmentCloud(const PointCloud &laser_cloud_in, PointCloud &laser_cloud_out)
{
    setParameter();

    // TODO: use voxel grid to filter noisy points
    // We first apply a voxel grid to the input point cloud P, in order to filter- out noise in voxels where there is not enough evidence for occupancy
    // pcl::VoxelGrid<Point> down_size_filter;
    // down_size_filter.setLeafSize(0.05, 0.05, 0.05);
    // down_size_filter.setInputCloud(boost::make_shared<PointCloud>(laser_cloud_in));
    // down_size_filter.filter(laser_cloud_out);

    projectCloud(laser_cloud_in);

    for (size_t i = 0; i < N_SCANS; i++)
        for (size_t j = 0; j < horizon_scans_; j++)
            if (label_mat_.at<int>(i,j) == 0) labelComponents(i, j);

    // labelConnectLine();

    laser_cloud_out.clear();
    for (size_t i = 0; i < N_SCANS; i++)
    {
        for (size_t j = 0; j < horizon_scans_; j++)
        {
            if ((label_mat_.at<int>(i, j) > 0) && (label_mat_.at<int>(i, j) != 999999))
            {
                laser_cloud_out.push_back(cloud_matrix_->points[j + i*horizon_scans_]);
            }
        }
    }
    // printf("input cloud size:%d, output cloud size:%d\n", laser_cloud_in.size(), laser_cloud_out.size());
}

void ImageSegmenter::labelComponents(int row, int col)
{
    float d1, d2, alpha, angle, dist;
    int from_indx, from_indy, this_indx, this_indy;
    bool line_count_flag[N_SCANS] = {false};

    queue_indx_[0] = row;
    queue_indy_[0] = col;
    queue_indx_last_negi_[0] = 0;
    queue_indy_last_negi_[0] = 0;
    queue_last_dis_[0] = 0;
    int queue_size = 1;
    int queue_start_ind = 0;
    int queue_end_ind = 1;

    all_pushed_indx_[0] = row;
    all_pushed_indy_[0] = col;
    int all_pushed_ind_size = 1;

    // find the neighbor connecting clusters in range image, bfs
    while (queue_size > 0)
    {
        from_indx = queue_indx_[queue_start_ind];
        from_indy = queue_indy_[queue_start_ind];
        --queue_size;
        ++queue_start_ind;
        label_mat_.at<int>(from_indx, from_indy) = label_count_;
        for (auto iter = neighbor_iterator_.begin(); iter != neighbor_iterator_.end(); ++iter)
        {
            this_indx = from_indx + iter->first;
            this_indy = from_indy + iter->second;
            if (this_indx < 0 || this_indx >= N_SCANS) continue;
            if (this_indy < 0) this_indy = horizon_scans_ - 1;
            if (this_indy >= horizon_scans_) this_indy = 0;
            if (label_mat_.at<int>(this_indx, this_indy) != 0) continue;

            // TODO: criteria of different objects
            d1 = std::max(range_mat_.at<float>(from_indx, from_indy),
                          range_mat_.at<float>(this_indx, this_indy));
            d2 = std::min(range_mat_.at<float>(from_indx, from_indy),
                          range_mat_.at<float>(this_indx, this_indy));
            dist = sqrt(d1*d1 + d2*d2 - 2*d1*d2*cos(alpha));
            alpha = iter->first == 0 ? segment_alphax_ : segment_alphay_;
            angle = atan2(d2*sin(alpha), (d1 - d2*cos(alpha)));
            if (angle > segment_theta)
            {
                queue_indx_[queue_end_ind] = this_indx;
                queue_indy_[queue_end_ind] = this_indy;
                queue_indx_last_negi_[queue_end_ind] = iter->first;
                queue_indy_last_negi_[queue_end_ind] = iter->second;
                queue_last_dis_[queue_end_ind] = dist;
                queue_size++;
                queue_end_ind++;

                label_mat_.at<int>(this_indx, this_indy) = label_count_;
                line_count_flag[this_indx] = true;

                all_pushed_indx_[all_pushed_ind_size] = this_indx;
                all_pushed_indy_[all_pushed_ind_size] = this_indy;
                all_pushed_ind_size++;
            }
            else if ((iter->second == 0) && (queue_indy_last_negi_[queue_start_ind] == 0)) // at the same beam
            {
                float dist_last = queue_last_dis_[queue_start_ind];
                if ((dist_last / dist <= 1.2) && ((dist_last / dist >= 0.8))) // inside a plane
                {
                    queue_indx_[queue_end_ind] = this_indx;
                    queue_indy_[queue_end_ind] = this_indy;
                    queue_indx_last_negi_[queue_end_ind] = iter->first;
                    queue_indy_last_negi_[queue_end_ind] = iter->second;
                    queue_last_dis_[queue_end_ind] = dist;
                    queue_size++;
                    queue_end_ind++;

                    label_mat_.at<int>(this_indx, this_indy) = label_count_;
                    line_count_flag[this_indx] = true;

                    all_pushed_indx_[all_pushed_ind_size] = this_indx;
                    all_pushed_indy_[all_pushed_ind_size] = this_indy;
                    all_pushed_ind_size++;
                }
            }
        }
    }
    bool feasible_segment = false;
    if (all_pushed_ind_size >= min_cluster_size_) // cluster_size > min_cluster_size_
    {
        feasible_segment = true;
    }
    else if (all_pushed_ind_size >= segment_valid_point_num_) // line_size > line_mini_size
    {
        int line_count = 0;
        for (size_t i = 0; i < N_SCANS; i++)
            if (line_count_flag[i]) line_count++;
        if (line_count >= segment_valid_line_num_) feasible_segment = true;
    }

    if (feasible_segment)
    {
        label_count_++;
    } else
    {
        for (size_t i = 0; i < all_pushed_ind_size; ++i)
        {
            label_mat_.at<int>(all_pushed_indx_[i], all_pushed_indy_[i]) = 999999;
        }
    }
}

// keep the long continuous lines, they are likely on large planes including ground
void ImageSegmenter::labelConnectLine()
{
    for (size_t i = 0; i < N_SCANS; i++)
    {
        size_t start_indx;
        for (start_indx = 0; start_indx < horizon_scans_ - 1; start_indx++)
            if (label_mat_.at<int>(i, start_indx) == 999999)
                break;
        size_t end_indx = start_indx;

        all_pushed_indx_[0] = start_indx;
        int all_pushed_ind_size = 1;
        while (end_indx <= horizon_scans_ - 2)
        {
            bool b_exit;
            if (label_mat_.at<int>(i, end_indx + 1) == -1) b_exit = false;
            else if (label_mat_.at<int>(i, end_indx + 1) != 999999) b_exit = true;
            else
            {
                // all_pushed_indx_[all_pushed_ind_size] = end_indx + 1;
                // all_pushed_ind_size++;
                // b_exit = false;
                float d1 = std::max(range_mat_.at<float>(i, end_indx),
                                    range_mat_.at<float>(i, end_indx + 1));
                float d2 = std::min(range_mat_.at<float>(i, end_indx),
                                    range_mat_.at<float>(i, end_indx + 1));
                float alpha = segment_alphax_;
                float angle = atan2(d2*sin(alpha), (d1 - d2*cos(alpha)));
                if (angle > segment_theta)
                {
                    all_pushed_indx_[all_pushed_ind_size] = end_indx + 1;
                    all_pushed_ind_size++;
                    b_exit = false;
                } else
                    b_exit = true;
            }
            if (end_indx == horizon_scans_ - 2) b_exit = true;
            if (b_exit)
            {
                if ((all_pushed_ind_size > min_line_size_) &&
                    ((1.0 * all_pushed_ind_size / (end_indx + 1 - start_indx)) > 0.5)) // TODO
                {
                    printf("label count: %d, size: %d, %f\%\n", label_count_, all_pushed_ind_size,
                        1.0 * all_pushed_ind_size / (end_indx + 1 - start_indx));
                    label_count_++;
                    for (size_t k = 0; k < all_pushed_ind_size; k++)
                    {
                        label_mat_.at<int>(i, all_pushed_indx_[k]) = label_count_;
                    }
                }
                start_indx = end_indx + 1;
                end_indx = start_indx;
                all_pushed_indx_[0] = start_indx;
                all_pushed_ind_size = 1;
            } else
            {
                end_indx++;
            }
        }
    }
}


/*
for (size_t i = 0; i < N_SCANS; i++)
{
    size_t step_cnt = 0;
    size_t j = 0;
    while (j < horizon_scans_ - 1)
    {
        if (label_mat_.at<int>(i, j) == 999999)
        {
            if (label_mat_.at<int>(i, j + 1) != -1) // non-point
            {
                d1 = std::max(range_mat_.at<float>(from_indx, from_indy),
                              range_mat_.at<float>(this_indx, this_indy));
                d2 = std::min(range_mat_.at<float>(from_indx, from_indy),
                              range_mat_.at<float>(this_indx, this_indy));
                alpha = iter->first == 0 ? segment_alphax_ : segment_alphay_;
                // the core value to check connecting
                angle = atan2(d2*sin(alpha), (d1 - d2*cos(alpha)));
                if (angle > segment_theta)
                {
                    all_pushed_indx_[all_pushed_ind_size] = j;
                    all_pushed_ind_size++;
                } else
                {
                    if (1.0 * all_pushed_ind_size / step_cnt > 0.5)
                    {
                        for (size_t k = 0; k < all_pushed_ind_size; k++)
                        {
                            label_mat_.at<int>(i, all_pushed_indx_[k]) = label_count_;
                        }
                    }
                }
            }
            step_cnt ++;
        }
    }
    for (size_t j = 0; j < horizon_scans_; j++)
    {
        if (label_mat_.at<int>(i, j) == 999999)
        {

        }
    }
}
*/

// region clustering
/*
pcl::search::Search<Point>::Ptr tree(new pcl::search::KdTree<Point>);
NormalCloudPtr normals(new NormalCloud);
pcl::NormalEstimation<Point, Normal> normal_estimator;
normal_estimator.setSearchMethod(tree);
normal_estimator.setInputCloud(boost::make_shared<PointCloud>(laser_cloud_in));
normal_estimator.setKSearch(5);
normal_estimator.compute(*normals);

pcl::RegionGrowing<Point, Normal> region_growing_estimator;
region_growing_estimator.setMinClusterSize(20);
region_growing_estimator.setMaxClusterSize(10000);
region_growing_estimator.setSearchMethod(tree);
region_growing_estimator.setNumberOfNeighbours(5);
region_growing_estimator.setSmoothModeFlag(true);
region_growing_estimator.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
region_growing_estimator.setCurvatureTestFlag(true);
region_growing_estimator.setCurvatureThreshold(1.0);
region_growing_estimator.setResidualTestFlag(false);

region_growing_estimator.setInputCloud(boost::make_shared<PointCloud>(laser_cloud_in));
region_growing_estimator.setInputNormals(normals);
std::vector<pcl::PointIndices> clusters_indices;
region_growing_estimator.extract(clusters_indices);
printf("cluster size:%d\n", clusters_indices.size());

for (std::vector<pcl::PointIndices>::const_iterator iter = clusters_indices.begin(); iter != clusters_indices.end(); ++iter)
{
    PointCloudPtr cluster(new PointCloud);
    pcl::copyPointCloud(laser_cloud_in, *iter, *cluster);
    laser_cloud_out += *cluster;
}
*/
