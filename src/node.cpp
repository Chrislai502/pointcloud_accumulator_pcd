// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.hpp>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <opencv2/opencv.hpp>
// #include <iostream>
// #include <filesystem>

// class DataProcessor : public rclcpp::Node {
//     public:
//         DataProcessor()
//         : Node("data_processor"), accumulated_clouds_(0)
//         {
//             this->declare_parameter<int>("n_accumulated_clouds", 8); //Default accummulate 10 clouds
//             this->get_parameter("n_accumulated_clouds", n_accumulated_clouds_);
//             this->declare_parameter<std::string>("output_path", "/home/chris/testing/lidar_cam_calib/scene_based/tools_ws/src/pointcloud_accumulator/output");
//             this->get_parameter("output_path", output_path_);

//             point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//                 "/luminar_front_points", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), 
//                 std::bind(&DataProcessor::point_cloud_callback, this, std::placeholders::_1)
//             );

//             image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
//                 "/vimba_front_left_center/image", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), 
//                 std::bind(&DataProcessor::image_callback, this, std::placeholders::_1)
//             );

//             merged_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
//         }

//     private:
//         void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//         {
//             pcl::PointCloud<pcl::PointXYZ> cloud;
//             pcl::fromROSMsg(*msg, cloud);

//             *merged_cloud_ += cloud;
//             ++accumulated_clouds_;

//             if (accumulated_clouds_ >= n_accumulated_clouds_)
//             {
//                 // Log the number of points in the merged cloud
//                 RCLCPP_INFO(this->get_logger(), "Merging %d point clouds with a total of %lu points.",
//                             accumulated_clouds_, merged_cloud_->size());

//                 // Generate Filename with accumulated_clouds_ count
//                 std::string filename = output_path_ + "merged_cloud_" + std::to_string(accumulated_clouds_) + ".pcd";

//                 pcl::io::savePCDFile(filename, *merged_cloud_);
//                 RCLCPP_INFO(this->get_logger(), "Saved merged cloud of %d point clouds.", accumulated_clouds_);
//             }

//             if (!latest_image_.empty())
//             {
//                 // Save the latest image
//                 std::string image_filename = output_path_ + "/latest_image.png";
//                 cv::imwrite(image_filename, latest_image_);
//                 RCLCPP_INFO(this->get_logger(), "Saved latest image to %s", image_filename.c_str());
//             }

//             // Shutdown the node
//             rclcpp::shutdown();
//         }

//         void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
//         {
//             cv_bridge::CvImagePtr cv_ptr;
//             try
//             {
//                 cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//                 latest_image_ = cv_ptr->image;
//             }
//             catch (cv_bridge::Exception& e)
//             {
//                 RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//                 return;
//             }
//         }

//         // Sensor Signatures
//         rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
//         rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
//         pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_;
//         cv::Mat latest_image_;
//         int n_accumulated_clouds_;
//         int accumulated_clouds_;
//         std::string output_path_;
// };

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<DataProcessor>();
//     rclcpp::spin(node);
//     return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>

class DataProcessor : public rclcpp::Node {
public:
    DataProcessor()
    : Node("data_processor"), accumulated_clouds_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing DataProcessor node.");

        this->declare_parameter<int>("n_accumulated_clouds", 8);
        this->get_parameter("n_accumulated_clouds", n_accumulated_clouds_);
        RCLCPP_INFO(this->get_logger(), "Number of clouds to accumulate: %d", n_accumulated_clouds_);

        this->declare_parameter<std::string>("output_path", "/home/chris/testing/lidar_cam_calib/scene_based/tools_ws/src/pointcloud_accumulator/output");
        this->get_parameter("output_path", output_path_);
        RCLCPP_INFO(this->get_logger(), "Output path set to: %s", output_path_.c_str());

        // Verify output directory exists
        if (!std::filesystem::exists(output_path_)) {
            RCLCPP_ERROR(this->get_logger(), "Output path does not exist: %s", output_path_.c_str());
            rclcpp::shutdown();
            return;
        }

        point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/luminar_front_points", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
            std::bind(&DataProcessor::point_cloud_callback, this, std::placeholders::_1)
        );

        // image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        //     "/vimba_front_left_center/image", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        //     std::bind(&DataProcessor::image_callback, this, std::placeholders::_1)
        // );

        merged_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        saved = false;
    }

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "PointCloud callback triggered.");

        if (accumulated_clouds_ < n_accumulated_clouds_)
        {
            pcl::PointCloud<pcl::PointXYZI> cloud;
            pcl::fromROSMsg(*msg, cloud);

            *merged_cloud_ += cloud;
            ++accumulated_clouds_;

            RCLCPP_INFO(this->get_logger(), "PointCloud merged. Total accumulated clouds: %d", accumulated_clouds_);
        }

        if (accumulated_clouds_ >= n_accumulated_clouds_ && !saved)
        {
            //DEBUG
            for (size_t i = 0; i < std::min(merged_cloud_->points.size(), size_t(10)); ++i) {
                const auto& point = merged_cloud_->points[i];
                std::cout << "Point " << i << ": x=" << point.x << ", y=" << point.y << ", z=" << point.z << ", intensity=" << point.intensity << std::endl;
            }
            //DEBUG


            RCLCPP_INFO(this->get_logger(), "Merging %d point clouds with a total of %lu points.",
                        accumulated_clouds_, merged_cloud_->size());

            std::string filename = output_path_ + "/merged_cloud_" + std::to_string(accumulated_clouds_) + ".pcd";

            if (pcl::io::savePCDFile(filename, *merged_cloud_) == -1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to save merged cloud to %s", filename.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Saved merged cloud of %d point clouds to %s", accumulated_clouds_, filename.c_str());
                saved = true;
            }

            // RCLCPP_INFO(this->get_logger(), "Shutting down the node after saving data.");
            rclcpp::shutdown();
        }

        // if (!latest_image_.empty())
        // {
        //     std::string image_filename = output_path_ + "/latest_image.png";
        //     if (cv::imwrite(image_filename, latest_image_)) {
        //         RCLCPP_INFO(this->get_logger(), "Saved latest image to %s", image_filename.c_str());
        //     } else {
        //         RCLCPP_ERROR(this->get_logger(), "Failed to save latest image to %s", image_filename.c_str());
        //     }
        // }
    }

    // void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Image callback triggered.");

    //     cv_bridge::CvImagePtr cv_ptr;
    //     try {
    //         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //         latest_image_ = cv_ptr->image;
    //         RCLCPP_INFO(this->get_logger(), "Image received and stored.");
    //     } catch (cv_bridge::Exception& e) {
    //         RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    //     }
    // }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud_;
    // cv::Mat latest_image_;
    int n_accumulated_clouds_;
    int accumulated_clouds_;
    std::string output_path_;
    bool saved;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting DataProcessor node.");
    auto node = std::make_shared<DataProcessor>();
    rclcpp::spin(node);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DataProcessor node finished.");
    rclcpp::shutdown();
    return 0;
}
