/******************************************************************************
 * Help functions for loading parameters from ROS Parameters Server
 *
 * ...
 *****************************************************************************/

#ifndef _PARAMETER_HPP_
#define _PARAMETER_HPP_

#include "./types/type.h"

namespace common {

    static VolumetricModelParams getVolumetricModelParams(const ros::NodeHandle& nh,
                                                          const std::string& ns_prefix)
    {
        VolumetricModelParams params;

        std::string volumetric_ns = ns_prefix + "/VolumetricModels";
        nh.param<bool>(volumetric_ns + "/use_car_volumetric_model", params.use_car_model, false);
        std::vector<double> volumetric_model(6, 0.);
        nh.getParam(volumetric_ns + "/car_volumetric_model", volumetric_model);
        params.model_car.model_type = CAR;
        params.model_car.l_min = volumetric_model[0]; params.model_car.l_max = volumetric_model[1];
        params.model_car.w_min = volumetric_model[2]; params.model_car.w_max = volumetric_model[3];
        params.model_car.h_min = volumetric_model[4]; params.model_car.h_max = volumetric_model[5];
        //common::displayModelInfo(model_car_);

        nh.param<bool>(volumetric_ns + "/use_human_volumetric_model", params.use_car_model, false);
        volumetric_model.resize(6, 0.);
        nh.getParam(volumetric_ns + "/human_volumetric_model", volumetric_model);
        params.model_car.model_type = PEDESTRIAN;
        params.model_human.l_min = volumetric_model[0]; params.model_human.l_max = volumetric_model[1];
        params.model_human.w_min = volumetric_model[2]; params.model_human.w_max = volumetric_model[3];
        params.model_human.h_min = volumetric_model[4]; params.model_human.h_max = volumetric_model[5];

        return params;
    }

    static ROIParams getRoiParams(const ros::NodeHandle& nh,
                                  const std::string& ns_prefix)
    {
        ROIParams params;

        const std::string ns = ns_prefix + "/roi";

        nh.getParam(ns + "/roi_type", params.type);

        nh.getParam(ns + "/roi_lidar_height_m", params.roi_lidar_height_m);
        // Horizontal range
        nh.getParam(ns + "/roi_radius_min_m", params.roi_radius_min_m);
        // "Cylinder" roi filter do not need `roi_radius_max_m`
        nh.param(ns + "/roi_radius_max_m", params.roi_radius_max_m, -1.0f);
        // Vertical range
        nh.getParam(ns + "/roi_height_below_m", params.roi_height_below_m);
        nh.getParam(ns + "/roi_height_above_m", params.roi_height_above_m);

        return params;
    }

    static SegmenterParams getSegmenterParams(const ros::NodeHandle& nh,
                                              const std::string& ns_prefix)
    {
        SegmenterParams params;
        const std::string ns = ns_prefix + "/Segmenters";

        // DoN Segmenter
        nh.getParam(ns + "/segmenter_type", params.segmenter_type);
        nh.getParam(ns + "/don_segmenter_small_scale", params.don_segmenter_small_scale);
        nh.getParam(ns + "/don_segmenter_large_scale", params.don_segmenter_large_scale);
        nh.getParam(ns + "/don_segmenter_range_threshold", params.don_segmenter_range_threshold);
        nh.getParam(ns + "/don_segmenter_ec_min_size", params.don_segmenter_ec_min_size);
        nh.getParam(ns + "/don_segmenter_ec_max_size", params.don_segmenter_ec_max_size);
        nh.getParam(ns + "/don_segmenter_ec_tolerance", params.don_segmenter_ec_tolerance);

        // Region Growing Segmenter
        nh.getParam(ns + "/rg_knn_for_normals", params.rg_knn_for_normals);
        nh.getParam(ns + "/rg_radius_for_normals", params.rg_radius_for_normals);
        nh.getParam(ns + "/rg_curvature_threshold", params.rg_curvature_threshold);
        nh.getParam(ns + "/rg_min_cluster_size", params.rg_min_cluster_size);
        nh.getParam(ns + "/rg_max_cluster_size", params.rg_max_cluster_size);
        nh.getParam(ns + "/rg_knn_for_growing", params.rg_knn_for_growing);
        nh.getParam(ns + "/rg_smoothness_threshold_deg", params.rg_smoothness_threshold_deg);

        // Region Euclidean Cluster non-ground Segmenter
        //nh.param<int>(ns + "/rec_region_size", params.rec_region_size, 14);
        nh.getParam(ns + "/rec_region_sizes", params.rec_region_sizes);
        params.rec_region_size = params.rec_region_sizes.size();
        nh.param<double>(ns + "/rec_region_initial_tolerance", params.rec_region_initial_tolerance, 0.2);
        nh.param<double>(ns + "/rec_region_delta_tolerance", params.rec_region_delta_tolerance, 0.2);
        nh.param<bool>(ns + "/rec_use_region_merge", params.rec_use_region_merge, false);
        nh.param<double>(ns + "/rec_region_merge_tolerance", params.rec_region_merge_tolerance, 0.);
        nh.param<int>(ns + "/rec_min_cluster_size", params.rec_min_cluster_size, 5);
        nh.param<int>(ns + "/rec_max_cluster_size", params.rec_max_cluster_size, 30000);

        // Euclidean Cluster non-ground Segmenter
        nh.param<double>(ns + "/ec_tolerance", params.ec_tolerance, 0.25);
        nh.param<int>(ns + "/ec_min_cluster_size", params.ec_min_cluster_size, 5);
        nh.param<int>(ns + "/ec_max_cluster_size", params.ec_max_cluster_size, 30000);

        // Ground Plane Fitting ground Segmenter
        nh.getParam(ns + "/gpf_sensor_model", params.gpf_sensor_model);
        nh.getParam(ns + "/gpf_sensor_height", params.gpf_sensor_height);
        nh.param<int>(ns + "/gpf_num_segment", params.gpf_num_segment, 1);
        nh.getParam(ns + "/gpf_num_iter", params.gpf_num_iter);
        nh.getParam(ns + "/gpf_num_lpr", params.gpf_num_lpr);
        nh.getParam(ns + "/gpf_th_lprs", params.gpf_th_lprs);
        nh.getParam(ns + "/gpf_th_seeds", params.gpf_th_seeds);
        nh.getParam(ns + "/gpf_th_gnds", params.gpf_th_gnds);

        // RANSAC ground Segmenter
        nh.param<double>(ns + "/sac_distance_threshold", params.sac_distance_threshold, 0.3);
        nh.param<int>(ns + "/sac_max_iteration", params.sac_max_iteration, 100);
        nh.param<double>(ns + "/sac_probability", params.sac_probability, 0.99);

        return params;
    }

    static FeatureExtractorParams getFeatureExtractorParams(const ros::NodeHandle& nh,
                                                            const std::string& ns_prefix)
    {
        FeatureExtractorParams params;
        const std::string ns = ns_prefix + "/Features";

        nh.getParam(ns + "/extractor_type", params.extractor_type);

        return params;
    }

    static ClassifierParams getClassfierParams(const ros::NodeHandle& nh,
                                               const std::string& ns_prefix)
    {
        ClassifierParams params;
        const std::string ns = ns_prefix + "/Classifier";

        nh.getParam(ns + "/classifier_type", params.classifier_type);

        nh.getParam(ns + "/classifier_model_path", params.classifier_model_path);

        //If true, save model in model specification&timestamps name
        nh.getParam(ns + "/classifier_save", params.classifier_save);

        nh.getParam(ns + "/classifier_max_num_samples", params.classifier_max_num_samples);

        //empty means no need to load, *.xml
        nh.getParam(ns + "/rf_model_filename", params.rf_model_filename);

        //empty means no need to load, *.model
        nh.getParam(ns + "/svm_model_filename", params.svm_model_filename);
        //*.range
        nh.getParam(ns + "/svm_range_filename", params.svm_range_filename);

        //----------------- Random Forest Classifier parameters
        nh.param<double>(ns + "/rf_threshold_to_accept_object", params.rf_threshold_to_accept_object, 1.0);
        // the depth of the tree
        nh.param<int>(ns + "/rf_max_depth", params.rf_max_depth, 25);
        // rf_min_sample_ratio*num_samples==>min sample count
        nh.param<double>(ns + "/rf_min_sample_ratio", params.rf_min_sample_ratio, 0.01);
        // regression accuracy: 0->N/A
        nh.param<double>(ns + "/rf_regression_accuracy", params.rf_regression_accuracy, 0);
        // compute surrogate split, false->no missing data
        nh.param<bool>(ns + "/rf_use_surrogates", params.rf_use_surrogates, false);
        // max number of categories (use sub-optimal algorithm for larger numbers)
        nh.param<int>(ns + "/rf_max_categories", params.rf_max_categories, 0);
        // weights of each classification for classes, commented for null
        nh.getParam(ns + "/rf_priors", params.rf_priors);
        // if true then variable importance will be calculated
        nh.param<bool>(ns + "/rf_calc_var_importance", params.rf_calc_var_importance, true);
        // number of variables randomly selected at node and used to find the best split(s)
        nh.param<int>(ns + "/rf_n_active_vars", params.rf_n_active_vars, 4);
        // max number of trees in the forest
        nh.param<int>(ns + "/rf_max_num_of_trees", params.rf_max_num_of_trees, 100);
        // forest accuracy
        nh.param<double>(ns + "/rf_accuracy", params.rf_accuracy, 0.01);

        //----------------- Random Forest Classifier parameters
        nh.getParam(ns + "/svm_threshold_to_accept_object", params.svm_threshold_to_accept_object);

        nh.getParam(ns + "/svm_find_the_best_training_parameters", params.svm_find_the_best_training_parameters);
        //feature range
        nh.getParam(ns + "/svm_feature_range_lower", params.svm_feature_range_lower);
        nh.getParam(ns + "/svm_feature_range_upper", params.svm_feature_range_upper);

        return params;
    }

    static TrackingWorkerParams getTrackingWorkerParams(const ros::NodeHandle& nh,
                                                        const std::string& ns_prefix)
    {
        TrackingWorkerParams params;
        const std::string ns = ns_prefix + "/TrackingWorker";

        //----------------- Matcher: tracker<->observed object association
        nh.getParam(ns + "/matcher_method_name", params.matcher_method_name);
        nh.getParam(ns + "/matcher_match_distance_maximum", params.matcher_match_distance_maximum);
        nh.getParam(ns + "/matcher_location_distance_weight", params.matcher_location_distance_weight);
        nh.getParam(ns + "/matcher_direction_distance_weight", params.matcher_direction_distance_weight);
        nh.getParam(ns + "/matcher_bbox_size_distance_weight", params.matcher_bbox_size_distance_weight);
        nh.getParam(ns + "/matcher_point_num_distance_weight", params.matcher_point_num_distance_weight);
        nh.getParam(ns + "/matcher_histogram_distance_weight", params.matcher_histogram_distance_weight);

        //----------------- Tracker
        //Tracker Filter setup
        nh.getParam(ns + "/filter_method_name", params.filter_method_name);
        nh.getParam(ns + "/filter_use_adaptive", params.filter_use_adaptive);
        nh.getParam(ns + "/filter_association_score_maximum", params.filter_association_score_maximum);
        nh.getParam(ns + "/filter_measurement_noise", params.filter_measurement_noise);
        nh.getParam(ns + "/filter_initial_velocity_noise", params.filter_initial_velocity_noise);
        nh.getParam(ns + "/filter_xy_propagation_noise", params.filter_xy_propagation_noise);
        nh.getParam(ns + "/filter_z_propagation_noise", params.filter_z_propagation_noise);
        nh.getParam(ns + "/filter_breakdown_threshold_maximum", params.filter_breakdown_threshold_maximum);
        //Basic Tracker setup
        nh.getParam(ns + "/tracker_cached_history_size_maximum", params.tracker_cached_history_size_maximum);
        nh.getParam(ns + "/tracker_consecutive_invisible_maximum", params.tracker_consecutive_invisible_maximum);
        nh.getParam(ns + "/tracker_visible_ratio_minimum", params.tracker_visible_ratio_minimum);
        nh.getParam(ns + "/tracker_acceleration_noise_maximum", params.tracker_acceleration_noise_maximum);
        nh.getParam(ns + "/tracker_speed_noise_maximum", params.tracker_speed_noise_maximum);

        //----------------- Tracking Objects collect conditions
        nh.getParam(ns + "/tracking_histogram_bin_size", params.tracking_histogram_bin_size);
        nh.getParam(ns + "/tracking_use_histogram_for_match", params.tracking_use_histogram_for_match);
        nh.getParam(ns + "/tracking_collect_age_minimum", params.tracking_collect_age_minimum);
        nh.getParam(ns + "/tracking_collect_consecutive_invisible_maximum",
                    params.tracking_collect_consecutive_invisible_maximum);

        return params;
    }

    static Parameters getParameters(const ros::NodeHandle& nh,
                                    const std::string& ns_prefix)
    {
        Parameters params;
        params.segmenter = getSegmenterParams(nh, ns_prefix);
        params.feature_extractor = getFeatureExtractorParams(nh, ns_prefix);
        params.classifier = getClassfierParams(nh, ns_prefix);

        return params;
    }
}

#endif /* _PARAMETER_HPP_ */