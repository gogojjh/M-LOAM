#include "cloud_visualizer.h"

using namespace common;

// *********************************
// PlaneNormalVisualizer
void PlaneNormalVisualizer::UpdateCloud(PointCloud::ConstPtr cloud,
                                        std::string cloud_name,
                                        std::vector<double> cloud_color)
{
    m_vis_.lock();
    // boost::mutex::scoped_lock lk(m_);
    //  DLOG(INFO) << ">>>>>>> update <<<<<<<";
    //  DLOG(INFO) << cloud->size();
    if (cloud->size() == 0)
    {
        // DLOG(INFO) << ">>>>>>> no points <<<<<<<";
        return;
    }

    // TODO
    viewer_->removePointCloud(cloud_name, 0);
    if (!viewer_->updatePointCloud(cloud, cloud_name))
    {
        viewer_->addPointCloud<Point>(cloud, cloud_name);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloud_name);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                 cloud_color[0],
                                                 cloud_color[1],
                                                 cloud_color[2],
                                                 cloud_name);
    }
    m_vis_.unlock();
}

void PlaneNormalVisualizer::UpdateCloudAndNormals(PointCloud::ConstPtr cloud,
                                                  NormalCloud::ConstPtr normals,
                                                  int ds_ratio,
                                                  std::string cloud_name,
                                                  std::string normals_name,
                                                  std::vector<double> cloud_color,
                                                  std::vector<double> normals_color)
{
    m_vis_.lock();
    // boost::mutex::scoped_lock lk(m_);
    //  DLOG(INFO) << ">>>>>>> update <<<<<<<";
    //  DLOG(INFO) << cloud->size();
    //  DLOG(INFO) << normals->size();

    if (cloud->size() == 0 || normals->size() == 0)
    {
        // DLOG(INFO) << ">>>>>>> no points <<<<<<<";
        return;
    }

    viewer_->removePointCloud(cloud_name, 0);
    if (!viewer_->updatePointCloud(cloud, cloud_name))
    {
        viewer_->addPointCloud<Point>(cloud, cloud_name);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                 cloud_color[0],
                                                 cloud_color[1],
                                                 cloud_color[2],
                                                 cloud_name);
    }
    viewer_->removePointCloud(normals_name, 0);
    viewer_->addPointCloudNormals<Point, Normal>(cloud, normals, ds_ratio, 0.5, normals_name); // cloud, normal, normal_ratio, length, name
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                           normals_color[0],
                                           normals_color[1],
                                           normals_color[2],
                                           normals_name);
    m_vis_.unlock();
}

void PlaneNormalVisualizer::UpdateLines(PointCloud::ConstPtr cloud1,
                                        PointCloud::ConstPtr cloud2,
                                        std::vector<double> line_color)
{
    m_vis_.lock();
    // boost::mutex::scoped_lock lk(m_);
    //  DLOG(INFO) << ">>>>>>> update <<<<<<<";
    int num_cloud1 = cloud1->size();
    int num_cloud2 = cloud2->size();
    if (num_cloud1 == 0 || num_cloud2 == 0 || num_cloud1 != num_cloud2)
    {
        // DLOG(INFO) << ">>>>>>> no points or sizes are not the same <<<<<<<";
        // LOG_IF(INFO, num_cloud1 != num_cloud2) << num_cloud1 << " != " << num_cloud2;
        return;
    }

    for (const auto line_name : line_names_)
    {
        viewer_->removeShape(line_name);
    }

    line_names_.clear();

    for (int i = 0; i < num_cloud1; ++i)
    {
        std::stringstream line_name_ss;
        line_name_ss << "line" << i;
        std::string line_name = line_name_ss.str();
        viewer_->addLine(cloud1->at(i), cloud2->at(i), line_name);
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                            line_color[0],
                                            line_color[1],
                                            line_color[2],
                                            line_name);
        line_names_.push_back(line_name);
    }
    m_vis_.unlock();
}

void PlaneNormalVisualizer::UpdatePlanes(const std::vector<Eigen::Vector4d> &plane_coeffs)
{
    m_vis_.lock();
    // boost::mutex::scoped_lock lk(m_);
    //  DLOG(INFO) << ">>>>>>> update <<<<<<<";
    int size = plane_coeffs.size();
    if (size == 0)
    {
        // DLOG(INFO) << ">>>>>>> no planes <<<<<<<";
        return;
    }

    for (const auto plane_name : plane_names_)
    {
        viewer_->removeShape(plane_name);
    }

    plane_names_.clear();
    for (int i = 0; i < size; ++i)
    {
        Eigen::Vector4d coeffs_eigen = plane_coeffs[i];
        std::stringstream plane_name_ss;
        plane_name_ss << "plane" << i;
        std::string plane_name = plane_name_ss.str();
        pcl::ModelCoefficients coeffs;
        coeffs.values.push_back(coeffs_eigen.x());
        coeffs.values.push_back(coeffs_eigen.y());
        coeffs.values.push_back(coeffs_eigen.z());
        coeffs.values.push_back(coeffs_eigen.w());
        viewer_->addPlane(coeffs, plane_name);
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.01, plane_name);
        plane_names_.push_back(plane_name);
    }
    m_vis_.unlock();
}

PlaneNormalVisualizer::PlaneNormalVisualizer()
{
    //  boost::mutex::scoped_lock lk(m);
    //  viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Debug Viewer");
    //  viewer->setBackgroundColor(0, 0, 0);
    //  viewer->addCoordinateSystem(1.0);
    //  viewer->addText("debugger by Kitkat7", 10, 10, "debugger text", 0);
    //  viewer->initCameraParameters();
    //  init = true;
}

void PlaneNormalVisualizer::Spin()
{
    m_vis_.lock();
    // std::mutex::scoped_lock lk(m_);
    viewer_ = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Debug Viewer");
    viewer_->setBackgroundColor(0, 0, 0);
    viewer_->addCoordinateSystem(1.0);
    viewer_->addText("debugger by developer", 10, 10, "debugger text", 0);
    viewer_->initCameraParameters();
    init_ = true;
    m_vis_.unlock();
    while (!viewer_->wasStopped())
    {
        // std::mutex::scoped_lock lk(m_);
        // DLOG(INFO) << ">>>>>>> spin <<<<<<<";
        // std::cout << ">>>>>>> spin <<<<<<<";
        m_vis_.lock();
        viewer_->spinOnce(100);
        m_vis_.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
}
