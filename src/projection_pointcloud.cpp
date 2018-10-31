/*
    Projection PointCloud (Velodyne and Realsense)

    author : Yudai Sadakuni
*/

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/common/float_image_utils.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <velodyne_camera_calibration/coloring.h>

template<typename T_p>
class Projection{
    private:
        ros::NodeHandle nh;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> sensor_fusion_sync_subs;
        message_filters::Subscriber<sensor_msgs::Image> image_sub;
        message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub;
        message_filters::Synchronizer<sensor_fusion_sync_subs> sensor_fusion_sync;

        ros::Publisher pub;

        tf::TransformListener listener;
        tf::StampedTransform  transform;
        bool flag;

    public:
        Projection();
        void Callback(const sensor_msgs::Image::ConstPtr&, const sensor_msgs::CameraInfo::ConstPtr&, const sensor_msgs::PointCloud2::ConstPtr&);
        void projection(const sensor_msgs::Image::ConstPtr, const sensor_msgs::CameraInfo::ConstPtr, const sensor_msgs::PointCloud2::ConstPtr);
        bool tflistener(std::string target_frame, std::string source_frame);
};

template<typename T_p>
Projection<T_p>::Projection()
    : nh("~"),
      image_sub(nh, "/image", 10), cinfo_sub(nh, "/cinfo", 10), lidar_sub(nh, "/lidar", 10),
      sensor_fusion_sync(sensor_fusion_sync_subs(10), image_sub, cinfo_sub, lidar_sub)
{
    sensor_fusion_sync.registerCallback(boost::bind(&Projection::Callback, this, _1, _2, _3));
    pub = nh.advertise<sensor_msgs::Image>("/projection", 10);
    flag = false;
}


// callback
template<typename T_p>
void Projection<T_p>::Callback(const sensor_msgs::Image::ConstPtr& image,
                             const sensor_msgs::CameraInfo::ConstPtr& cinfo,
                             const sensor_msgs::PointCloud2::ConstPtr& pc2)
{

    if(!flag) tflistener(image->header.frame_id, pc2->header.frame_id);
    projection(image, cinfo, pc2);
}

// transform listener
template<typename T_p>
bool Projection<T_p>::tflistener(std::string target_frame, std::string source_frame)
{
    ros::Time time = ros::Time(0);
    try{
        listener.waitForTransform(target_frame, source_frame, time, ros::Duration(4.0));
        listener.lookupTransform(target_frame, source_frame,  time, transform);
        return true;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(4.0).sleep();
        return false;
    }
}

// main function
template<typename T_p>
void Projection<T_p>::projection(const sensor_msgs::Image::ConstPtr image,
                                 const sensor_msgs::CameraInfo::ConstPtr cinfo,
                                 const sensor_msgs::PointCloud2::ConstPtr pc2)
{
    typename pcl::PointCloud<T_p>::Ptr cloud(new pcl::PointCloud<T_p>);
    pcl::fromROSMsg(*pc2, *cloud);
            
    // transform pointcloud from lidar_frame to camera_frame
    tf::Transform tf;
    tf.setOrigin(transform.getOrigin());
    tf.setRotation(transform.getRotation());
    typename pcl::PointCloud<T_p>::Ptr trans_cloud(new pcl::PointCloud<T_p>);
    pcl_ros::transformPointCloud(*cloud, *trans_cloud, tf);

    //cv_bridge
    std::cout<<"cv_bridge"<<std::endl;
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
        cv_img_ptr = cv_bridge::toCvShare(image);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_bridge::toCvShare(image)->image;
    
    // Realsense Data is saved BGR. change BGR to RGB
    cv::Mat rgb_image;
    cv::cvtColor(cv_image ,rgb_image, CV_BGR2RGB);

    // set PinholeCameraModel
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cinfo);

    // Projection Step
    for(typename pcl::PointCloud<T_p>::iterator pt=trans_cloud->points.begin(); pt<trans_cloud->points.end(); pt++)
    {
        if((*pt).z<0) continue;

        cv::Point3d pt_cv((*pt).x, (*pt).y, (*pt).z);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);

        if(uv.x>0 && uv.x < rgb_image.cols && uv.y > 0 && uv.y < rgb_image.rows)
        {
            double range = sqrt( pow((*pt).x, 2.0) + pow((*pt).y, 2.0) + pow((*pt).z, 2.0));
            COLOUR c = GetColour(int(range/20*255.0), 0, 255);
            cv::circle(rgb_image, uv, 1, cv::Scalar(int(255*c.b),int(255*c.g),int(255*c.r)), -1);
            // rgb_image.at<cv::Vec3b>(uv)[0] = 255*c.r;
            // rgb_image.at<cv::Vec3b>(uv)[1] = 255*c.g;
            // rgb_image.at<cv::Vec3b>(uv)[2] = 255*c.b;
        }
    }

    sensor_msgs::ImagePtr projection_image;
    projection_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_image).toImageMsg();
    projection_image->header.frame_id = image->header.frame_id;
    projection_image->header.stamp = ros::Time::now();
    pub.publish(projection_image);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "projection_pointcloud");

    Projection<pcl::PointXYZI> cr;

    ros::spin();

    return 0;
}
