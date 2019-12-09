#include "image_segmenter.h"

using namespace common;

const float segment_theta = 60.0 / 180.0 * M_PI;
const int segment_valid_point_num = 5;
const int segment_valid_line_num = 3;

ImageSegmenter::ImageSegmenter()
{
    std::pair<int8_t, int8_t> neighbor;
    neighbor.first = -1; neighbor.second =  0; neighbor_iterator_.push_back(neighbor);
    neighbor.first =  0; neighbor.second =  1; neighbor_iterator_.push_back(neighbor);
    neighbor.first =  0; neighbor.second = -1; neighbor_iterator_.push_back(neighbor);
    neighbor.first =  1; neighbor.second =  0; neighbor_iterator_.push_back(neighbor);
}

void ImageSegmenter::setScanParam(const int &horizon_scans, const int &min_cluster_size)
{
    horizon_scans_ = horizon_scans;
    ang_res_x_ = 360.0 / horizon_scans_;
    ang_res_y_ = 2.0; // TODO: adaptive to different scan number
    ang_bottom_ = 15.0+0.1;
    segment_alphax_ = ang_res_x_ / 180.0 * M_PI;
    segment_alphay_ = ang_res_y_ / 180.0 * M_PI;

    min_cluster_size_ = min_cluster_size;
}

void ImageSegmenter::setParameter()
{
    cloud_matrix_.reset(new PointCloud());
    cloud_matrix_->points.resize(N_SCANS * horizon_scans_);

    range_mat_ = cv::Mat(N_SCANS, horizon_scans_, CV_32F, cv::Scalar::all(FLT_MAX));
    label_mat_ = cv::Mat(N_SCANS, horizon_scans_, CV_32S, cv::Scalar::all(0));
    label_count_ = 1;

    all_pushed_indx = new uint16_t[N_SCANS*horizon_scans_];
    all_pushed_indy = new uint16_t[N_SCANS*horizon_scans_];

    queue_indx = new uint16_t[N_SCANS*horizon_scans_];
    queue_indy = new uint16_t[N_SCANS*horizon_scans_];
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
        // if ((horizon_angle <= 180) && (horizon_angle > 0))
        //     column_id = round(horizon_angle / ang_res_x_)
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
    printf("points in range image: %d\n", cloud_size);
    for (size_t i = 0; i < N_SCANS; i++)
        for (size_t j = 0; j < horizon_scans_; j++)
            if (range_mat_.at<float>(i, j) == FLT_MAX) label_mat_.at<int>(i, j) = -1;
}

void ImageSegmenter::segmentCloud(const PointCloud &laser_cloud_in, PointCloud &laser_cloud_out)
{
    setParameter();
    projectCloud(laser_cloud_in);

    for (size_t i = 0; i < N_SCANS; i++)
        for (size_t j = 0; j < horizon_scans_; j++)
            if (label_mat_.at<int>(i,j) == 0) labelComponents(i, j);

    laser_cloud_out.clear();
    for (size_t i = 0; i < N_SCANS; ++i)
    {
        for (size_t j = 0; j < horizon_scans_; ++j)
        {
            if ((label_mat_.at<int>(i, j) > 0) && (label_mat_.at<int>(i, j) != 999999))
            {
                laser_cloud_out.push_back(cloud_matrix_->points[j + i*horizon_scans_]);
            }
        }
    }
    std::cerr << "raw cloud size: " << laser_cloud_in.size() << std::endl;
    std::cerr << "out cloud size: " << laser_cloud_out.size() << std::endl;
}

void ImageSegmenter::labelComponents(int row, int col)
{
    float d1, d2, alpha, angle;
    int from_indx, from_indy, this_indx, this_indy;
    bool line_count_flag[N_SCANS] = {false};

    queue_indx[0] = row;
    queue_indy[0] = col;
    int queue_size = 1;
    int queue_start_ind = 0;
    int queue_end_ind = 1;

    all_pushed_indx[0] = row;
    all_pushed_indy[0] = col;
    int all_pushed_ind_size = 1;

    // find the neighbor connecting clusters in range image, bfs
    while (queue_size > 0)
    {
        from_indx = queue_indx[queue_start_ind];
        from_indy = queue_indy[queue_start_ind];
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
            d1 = std::max(range_mat_.at<float>(from_indx, from_indy),
                          range_mat_.at<float>(this_indx, this_indy));
            d2 = std::min(range_mat_.at<float>(from_indx, from_indy),
                          range_mat_.at<float>(this_indx, this_indy));
            alpha = iter->first == 0 ? segment_alphax_ : segment_alphay_;
            // the core value to check connecting
            angle = atan2(d2*sin(alpha), (d1 - d2*cos(alpha)));
            if (angle > segment_theta)
            {
                queue_indx[queue_end_ind] = this_indx;
                queue_indy[queue_end_ind] = this_indy;
                queue_size++;
                queue_end_ind++;

                label_mat_.at<int>(this_indx, this_indy) = label_count_;
                line_count_flag[this_indx] = true;

                all_pushed_indx[all_pushed_ind_size] = this_indx;
                all_pushed_indy[all_pushed_ind_size] = this_indy;
                all_pushed_ind_size++;
            }
        }
    }
    bool feasible_segment = false;
    if (all_pushed_ind_size >= min_cluster_size_) // cluster_size > min_cluster_size_
    {
        feasible_segment = true;
    }
    else if (all_pushed_ind_size >= segment_valid_point_num) // line_size > line_mini_size
    {
        int line_count = 0;
        for (size_t i = 0; i < N_SCANS; i++)
            if (line_count_flag[i]) line_count++;
        if (line_count >= segment_valid_line_num) feasible_segment = true;
    }

    if (feasible_segment)
    {
        label_count_++;
    } else
    {
        for (size_t i = 0; i < all_pushed_ind_size; ++i)
        {
            label_mat_.at<int>(all_pushed_indx[i], all_pushed_indy[i]) = 999999;
        }
    }
}
