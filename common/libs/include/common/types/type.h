#ifndef _TYPE_H_
#define _TYPE_H_

#include <iostream>

#include <pcl/point_types.h>            /* pcl::PointXYZ */
#include <pcl/point_cloud.h>            /* pcl::PointCloud */

#include <Eigen/Core>
#include <vector>
#include <tuple>

namespace common
{
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;
    typedef PointCloud::Ptr PointCloudPtr;
    typedef PointCloud::ConstPtr PointCloudConstPtr;

    typedef pcl::PointXYZI PointI;
    typedef pcl::PointCloud<PointI> PointICloud;
    typedef PointICloud::Ptr PointICloudPtr;
    typedef PointICloud::ConstPtr PointICloudConstPtr;

    typedef pcl::PointNormal PointN;
    typedef pcl::PointCloud<PointN> PointNCloud;
    typedef PointNCloud::Ptr PointNCloudPtr;
    typedef PointNCloud::ConstPtr PointNCloudConstPtr;

    typedef pcl::Normal Normal;
    typedef pcl::PointCloud<Normal> NormalCloud;
    typedef NormalCloud::Ptr NormalCloudPtr;
    typedef NormalCloud::ConstPtr NormalCloudConstPtr;

    // using double type to define x, y, z.
    struct PointD {
        double x;
        double y;
        double z;
        uint8_t intensity;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    } EIGEN_ALIGN16;

    typedef pcl::PointCloud<PointD> PointDCloud;
    typedef PointDCloud::Ptr PointDCloudPtr;
    typedef PointDCloud::ConstPtr PointDCloudConstPtr;

    ///@brief Object classify type
    typedef enum {
        PEDESTRIAN = 0x01,
        CAR = 0x02,
        TRUCK = 0x04,
        CYCLIST = 0x10,
        DONTCARE = 0x20,
        CARE = PEDESTRIAN | CAR | TRUCK | CYCLIST,
        PEDESTRIAN_CAR = PEDESTRIAN | CAR
    } ObjectType;

    typedef uint32_t IdType;
    #define ID_MAX UINT_MAX
    //poses array
    typedef std::vector<Eigen::Vector3f> Trajectory;
    //trajectory's tracker id and corresponding period
    typedef std::tuple<IdType, double> FixedTrajectory;
    //trajectory's period and corresponding segments' id array
    typedef std::tuple<double, std::vector<IdType>> FixedTrajectory2;

    /**
     * @brief car/human-like volumetric model is used to filter out over- and under-segmented clusters
     */
    typedef struct {
        ObjectType model_type;
        double l_min, l_max;
        double w_min, w_max;
        double h_min, h_max;
    } VolumetricModel;

    struct VolumetricModelParams {
        bool use_car_model;
        VolumetricModel model_car;
        bool use_human_model;
        VolumetricModel model_human;
    };

    /*------------------------------ Parameter type ------------------------------*/
    struct ROIParams {
        std::string type;

        float roi_lidar_height_m;
        // Horizontal range
        float roi_radius_min_m;
        float roi_radius_max_m;
        // Vertical range
        float roi_height_below_m;
        float roi_height_above_m;
    }; // struct ROIParams for ROI filter parameters

    struct SegmenterParams {
        std::string segmenter_type = "DoNSegmenter";

        // DonSegmenter parameters.
        double don_segmenter_small_scale = 0.5;
        double don_segmenter_large_scale = 2;
        double don_segmenter_range_threshold = 0.2;
        int don_segmenter_ec_min_size = 50;
        int don_segmenter_ec_max_size = 100000;
        double don_segmenter_ec_tolerance = 1.0;

        // Region growing parameters.
        int rg_knn_for_normals = 0;
        double rg_radius_for_normals = 0.0f;
        double rg_curvature_threshold = 0.0f;
        int rg_min_cluster_size;
        int rg_max_cluster_size;
        int rg_knn_for_growing;
        double rg_smoothness_threshold_deg;

        // Region Euclidean Cluster Segmenter
        int rec_region_size;
        std::vector<int> rec_region_sizes;
        double rec_region_initial_tolerance;
        double rec_region_delta_tolerance;
        int rec_min_cluster_size;
        int rec_max_cluster_size;
        bool rec_use_region_merge;
        double rec_region_merge_tolerance;

        // Euclidean segmenter parameters.
        double ec_tolerance;
        int ec_max_cluster_size;
        int ec_min_cluster_size;

        // Ground Plane Fitting
        int gpf_sensor_model = 64;
        double gpf_sensor_height = 1.73;
        //fitting multiple planes
        int gpf_num_segment;
        //number of iterations
        int gpf_num_iter;
        //number of points used to estimate the lowest point representative(LPR)
        //double of senser model???
        int gpf_num_lpr;
        double gpf_th_lprs;
        //threshold for points to be considered initial seeds
        double gpf_th_seeds;
        //ground points threshold distance from the plane <== large to guarantee safe removal
        double gpf_th_gnds;

        // Ground RANSAC Segmenter
        double sac_distance_threshold;
        int sac_max_iteration;
        double sac_probability;
    }; // struct SegmenterParams

    struct FeatureExtractorParams {
        std::string extractor_type;
        //std::vector<std::string> descriptor_types;

        /*int n_nearest_neighbours;
        bool enable_two_stage_retrieval;
        int knn_feature_dim;
        bool apply_hard_threshold_on_feature_distance;
        double feature_distance_threshold;

        bool normalize_eigen_for_knn;
        bool normalize_eigen_for_hard_threshold;
        std::vector<double> max_eigen_features_values;*/

    }; // struct FeatureExtractorParams

    struct ClassifierParams {
        std::string classifier_type;

        std::string classifier_model_path;
        //If true, save model in model specification&timestamps name
        bool classifier_save;

        int classifier_max_num_samples;

        //empty means no need to load, *.xml
        std::string rf_model_filename;

        //empty means no need to load, *.model
        std::string svm_model_filename;
        //*.range
        std::string svm_range_filename;

        // OpenCv random forest parameters.
        double rf_threshold_to_accept_object;
        //the depth of the tree
        int rf_max_depth;
        //rf_min_sample_ratio*num_samples==>min sample count
        double rf_min_sample_ratio;
        //regression accuracy: 0->N/A
        double rf_regression_accuracy;
        //compute surrogate split, false->no missing data
        bool rf_use_surrogates;
        // max number of categories (use sub-optimal algorithm for larger numbers)
        int rf_max_categories;
        std::vector<double> rf_priors;
        //if true then variable importance will be calculated
        bool rf_calc_var_importance;
        //number of variables randomly selected at node and used to find the best split(s)
        int rf_n_active_vars;
        //max number of trees in the forest
        int rf_max_num_of_trees;
        //forest accuracy
        double rf_accuracy;

        double svm_threshold_to_accept_object;
        bool svm_find_the_best_training_parameters;
        //svm features normalized range
        double svm_feature_range_lower;
        double svm_feature_range_upper;

    }; // struct ClassifierParams

    struct TrackingWorkerParams {

        //----------------- Matcher: tracker<->observed object association
        std::string matcher_method_name = "hungarian_matcher";
        float matcher_match_distance_maximum = 4.0;
        float matcher_location_distance_weight = 0.6f;
        float matcher_direction_distance_weight = 0.2f;
        float matcher_bbox_size_distance_weight = 0.1f;
        float matcher_point_num_distance_weight = 0.1f;
        float matcher_histogram_distance_weight = 0.5f;

        //----------------- Tracker
        //Tracker Filter setup
        std::string filter_method_name = "robust_kalman_filter";
        bool filter_use_adaptive = false;
        double filter_association_score_maximum = matcher_match_distance_maximum;
        float filter_measurement_noise = 0.4f;
        float filter_initial_velocity_noise = 5.0f;
        float filter_xy_propagation_noise = 10.0f;
        float filter_z_propagation_noise = 10.0f;
        float filter_breakdown_threshold_maximum = 10.0;
        //Basic Tracker setup
        int tracker_cached_history_size_maximum = 5;
        int tracker_consecutive_invisible_maximum = 3;
        float tracker_visible_ratio_minimum = 0.6;
        float tracker_acceleration_noise_maximum = 5;
        float tracker_speed_noise_maximum = 0.4;

        //----------------- Tracking Objects collect conditions
        bool tracking_use_histogram_for_match = false;
        float tracking_histogram_bin_size = 10.;
        int tracking_collect_age_minimum = 1;
        int tracking_collect_consecutive_invisible_maximum = 0;

    }; // struct TrackingWorkerParams

    struct Parameters {
        SegmenterParams segmenter;
        FeatureExtractorParams feature_extractor;
        ClassifierParams classifier;
    }; // struct Parameters, containing all-kind Params
}
#endif /* _TYPE_H_ */
