/*
    Coloring PointCloud (Velodyne and Realsense)

    author : Yudai Sadakuni
*/

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

template<typename T_p>
class Coloring{
    private:
        ros::NodeHandle nh;

        ros::Subscriber lidar_sub;
        
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> camera_sync_subs;
        message_filters::Subscriber<sensor_msgs::Image> image_sub;
        message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub;
        message_filters::Synchronizer<camera_sync_subs> camera_sync;

        ros::Publisher pub;

        sensor_msgs::Image::ConstPtr image_;
        sensor_msgs::CameraInfo::ConstPtr cinfo_;
        sensor_msgs::PointCloud2::ConstPtr pc2_;

    public:
        Coloring();
        void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr&);
        void cameraCallback(const sensor_msgs::Image::ConstPtr&, const sensor_msgs::CameraInfo::ConstPtr&);
        void coloring(const sensor_msgs::Image::ConstPtr, const sensor_msgs::CameraInfo::ConstPtr, const sensor_msgs::PointCloud2::ConstPtr);
};

template<typename T_p>
Coloring<T_p>::Coloring()
    : nh("~"),
      image_sub(nh, "/image", 10), cinfo_sub(nh, "/cinfo", 10),
      camera_sync(camera_sync_subs(10), image_sub, cinfo_sub)
{
    lidar_sub = nh.subscribe("/lidar", 10, &Coloring::lidarCallback, this);
    camera_sync.registerCallback(boost::bind(&Coloring::cameraCallback, this, _1, _2));
    pub = nh.advertise<sensor_msgs::PointCloud2>("/colord_cloud", 10);
}

template<typename T_p>
void Coloring<T_p>::cameraCallback(const sensor_msgs::Image::ConstPtr& image,
                                   const sensor_msgs::CameraInfo::ConstPtr& cinfo)
{
    image_ = image;
    cinfo_ = cinfo;
}

template<typename T_p>
void Coloring<T_p>::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& pc2)
{
    pc2_ = pc2;
    coloring(image_, cinfo_, pc2_);
}

template<typename T_p>
void Coloring<T_p>::coloring(const sensor_msgs::Image::ConstPtr image,
                             const sensor_msgs::CameraInfo::ConstPtr cinfo,
                             const sensor_msgs::PointCloud2::ConstPtr pc2)
{
    typename pcl::PointCloud<T_p>::Ptr cloud(new pcl::PointCloud<T_p>);
    pcl::fromROSMsg(*pc2, *cloud);

    // tflistener
    tf::TransformListener listener;
    tf::StampedTransform  transform;
    try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform(pc2->header.frame_id, image->header.frame_id, now, ros::Duration(1.0));
        listener.lookupTransform(pc2->header.frame_id, image->header.frame_id,  now, transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    // transform pointcloud from lidar_frame to camera_frame
    tf::Transform tf;
    tf.setOrigin(transform.getOrigin());
    tf.setRotation(transform.getRotation());
    typename pcl::PointCloud<T_p>::Ptr trans_cloud(new pcl::PointCloud<T_p>);
    pcl_ros::transformPointCloud(*cloud, *trans_cloud, tf);

    //cv_bridge
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
        cv_img_ptr = cv_bridge::toCvShare(image);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_bridge::toCvShare(image)->image;

    // set PinholeCameraModel
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cinfo);

    // Coloring Step
    typename pcl::PointCloud<T_p>::Ptr colored_cloud(new pcl::PointCloud<T_p>);
    *colored_cloud = *trans_cloud; 
    for(typename pcl::PointCloud<T_p>::iterator pt=colored_cloud->points.begin(); pt<colored_cloud->points.end(); pt++)
    {
        if((*pt).x<0) continue;

        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);

        if(uv.x>0 && uv.x < cv_image.cols && uv.y > 0 && uv.y < cv_image.rows)
        {
            (*pt).b = cv_image.at<cv::Vec3b>(uv)[0];
            (*pt).g = cv_image.at<cv::Vec3b>(uv)[1];
            (*pt).r = cv_image.at<cv::Vec3b>(uv)[2];
        }
    }

    // transform pointcloud from camera_frame to lidar_frame
    typename pcl::PointCloud<T_p>::Ptr output_cloud(new pcl::PointCloud<T_p>);
    pcl_ros::transformPointCloud(*colored_cloud, *output_cloud, tf.inverse());
    
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*output_cloud, output);
    output.header.frame_id = pc2->header.frame_id;
    output.header.stamp = pc2->header.stamp;
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coloring_pointcloud");

    Coloring<pcl::PointXYZRGB> cr;

    ros::spin();

    return 0;
}
