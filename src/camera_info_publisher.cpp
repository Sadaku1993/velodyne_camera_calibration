#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/CameraInfo.h>

class CameraInfo{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;

    public:
        CameraInfo();
        void camerainfo();

        int height;
        int width;
        std::string distortion_model;
        std::string frame_id;

        std::vector<double> D;
        std::vector<double> K;
        std::vector<double> R;
        std::vector<double> P;

        int binning_x;
        int binning_y;

        int roi_x_offset;
        int roi_y_offset;
        int roi_height;
        int roi_width;
        bool roi_do_rectify;

};

CameraInfo::CameraInfo()
    : nh("~")
{
    pub = nh.advertise<sensor_msgs::CameraInfo>("/camera_info", 1);

    nh.getParam("height", height);
    nh.getParam("width", width);
    nh.getParam("distortion_model", distortion_model);
    nh.getParam("header/frame_id", frame_id);

    nh.getParam("D", D);
    nh.getParam("K", K);
    nh.getParam("R", R);
    nh.getParam("P", P);

    nh.getParam("binning_x", binning_x);
    nh.getParam("binning_y", binning_y);

    nh.getParam("roi/x_offset", roi_x_offset);
    nh.getParam("roi/y_offset", roi_y_offset);
    nh.getParam("roi/height", roi_height);
    nh.getParam("roi/width", roi_width);
    nh.getParam("roi/do_rectify", roi_do_rectify);
}

void CameraInfo::camerainfo()
{
    sensor_msgs::CameraInfo cinfo;
    cinfo.header.stamp = ros::Time::now();
    cinfo.header.frame_id = frame_id;
    cinfo.height = height;
    cinfo.width = width;
    cinfo.distortion_model = distortion_model;
    cinfo.D = D;

    for(size_t i=0;i<cinfo.K.size();i++)
        cinfo.K[i] = K[i];

    for(size_t i=0;i<cinfo.R.size();i++)
        cinfo.R[i] = R[i];

    for(size_t i=0;i<cinfo.P.size();i++)
        cinfo.P[i] = P[i];

    cinfo.binning_x = binning_x;
    cinfo.binning_y = binning_y;

    cinfo.roi.x_offset = roi_x_offset;
    cinfo.roi.y_offset = roi_y_offset;
    cinfo.roi.height = roi_height;
    cinfo.roi.width = roi_width;
    cinfo.roi.do_rectify = roi_do_rectify;

    pub.publish(cinfo);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_info_publisher");

    CameraInfo ci;

    ros::Rate rate(20);
    while(ros::ok()){
        ci.camerainfo();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
