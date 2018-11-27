#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <string>
#include "calibration.hpp"

class ZedNativeHandler
{
  public:
    ZedNativeHandler();
    ~ZedNativeHandler();
    bool Init();
    void Run();

  private:
    void FillCamInfo();
    void PublishCamInfo(sensor_msgs::CameraInfoPtr camInfoMsg, ros::Publisher pubCamInfo, ros::Time t);
    void PublishImage(cv::Mat img, image_transport::Publisher &pubImg, std::string imgFrameId, ros::Time t);

    ros::NodeHandle mNH;
    ros::NodeHandle mPrivateNH;

    image_transport::Publisher mPubLeft;
    image_transport::Publisher mPubRawLeft;
    image_transport::Publisher mPubRight;
    image_transport::Publisher mPubRawRight;

    ros::Publisher mPubLeftCamInfo;
    ros::Publisher mPubRightCamInfo;
    sensor_msgs::CameraInfoPtr mLeftCamInfoMsg;
    sensor_msgs::CameraInfoPtr mRightCamInfoMsg;

    std::string mRightCamOptFrameId;
    std::string mLeftCamOptFrameId;

    int mCamDeviceID; // cam index, usually 0 but on a laptop the internal camera might be at 0 and the zed at 1
    int mCamResWidth;
    int mCamResHeight;
    int mCamFps;
    int mPublishGray;
    int mPublishRectified;
    int mPublishRaw;
    unsigned int mZedSN;

    cv::Mat mCamDistLeft;
    cv::Mat mCamMatrixLeftRaw;
    cv::Mat mCamMatrixLeftRect;
    cv::Mat mCamDistRight;
    cv::Mat mCamMatrixRightRaw;
    cv::Mat mCamMatrixRightRect;
    cv::Mat mCamLeftMapX;
    cv::Mat mCamLeftMapY;
    cv::Mat mCamRightMapX;
    cv::Mat mCamRightMapY;

    cv::VideoCapture mVideoCap;
};

ZedNativeHandler::ZedNativeHandler()
{
    mCamDistLeft = cv::Mat::zeros(5, 1, CV_64F);
    mCamDistRight = cv::Mat::zeros(5, 1, CV_64F);
}

ZedNativeHandler::~ZedNativeHandler()
{
}

bool ZedNativeHandler::Init()
{
    mNH = ros::NodeHandle();
    mPrivateNH = ros::NodeHandle("~");

    int dumb_sn = 0;
    if (!mPrivateNH.getParam("serial_number", dumb_sn))
    {
        ROS_WARN("ZED serial number parameter must be set.");
        return false;
    }
    mZedSN = dumb_sn;
    ROS_INFO_STREAM("ZED serial number: " << mZedSN);

    mPrivateNH.param<int>("cam_id", mCamDeviceID, 0);
    mPrivateNH.param<int>("resolution_width", mCamResWidth, 1280);
    mPrivateNH.param<int>("resolution_height", mCamResHeight, 720);
    mPrivateNH.param<int>("cam_fps", mCamFps, 30);
    mPrivateNH.param<int>("publish_gray", mPublishGray, 1);
    mPrivateNH.param<int>("publish_rectified", mPublishRectified, 1);
    mPrivateNH.param<int>("publish_raw", mPublishRaw, 0);
    mPrivateNH.param<std::string>("left_camera_optical_frame", mLeftCamOptFrameId, "left_camera_optical_frame");
    mPrivateNH.param<std::string>("right_camera_optical_frame", mRightCamOptFrameId, "right_camera_optical_frame");

    std::string img_topic = mPublishGray ? "image_rect_gray" : "image_rect_color";
    std::string img_raw_topic = mPublishGray ? "image_raw_gray" : "image_raw_color";
    std::string left_topic = "left/" + img_topic;
    std::string left_raw_topic = "left/" + img_raw_topic;
    std::string left_cam_info_topic = "left/camera_info";
    std::string right_topic = "right/" + img_topic;
    std::string right_raw_topic = "right/" + img_raw_topic;
    std::string right_cam_info_topic = "right/camera_info";
    mPrivateNH.getParam("left_topic", left_topic);
    mPrivateNH.getParam("left_raw_topic", left_raw_topic);
    mPrivateNH.getParam("left_cam_info_topic", left_cam_info_topic);
    mPrivateNH.getParam("right_topic", right_topic);
    mPrivateNH.getParam("right_raw_topic", right_raw_topic);
    mPrivateNH.getParam("right_cam_info_topic", right_cam_info_topic);

    mLeftCamInfoMsg = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo());
    mRightCamInfoMsg = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo());

    image_transport::ImageTransport it(mNH);

    if (mPublishRectified)
    {
        mPubLeft = it.advertise(left_topic, 1);
        ROS_INFO_STREAM("Advertised on topic " << left_topic);
        mPubRight = it.advertise(right_topic, 1);
        ROS_INFO_STREAM("Advertised on topic " << right_topic);
    }

    if (mPublishRaw)
    {
        mPubRawLeft = it.advertise(left_raw_topic, 1);
        ROS_INFO_STREAM("Advertised on topic " << left_raw_topic);
        mPubRawRight = it.advertise(right_raw_topic, 1);
        ROS_INFO_STREAM("Advertised on topic " << right_raw_topic);
    }

    mPubLeftCamInfo = mNH.advertise<sensor_msgs::CameraInfo>(left_cam_info_topic, 1);
    ROS_INFO_STREAM("Advertised on topic " << left_cam_info_topic);
    mPubRightCamInfo = mNH.advertise<sensor_msgs::CameraInfo>(right_cam_info_topic, 1);
    ROS_INFO_STREAM("Advertised on topic " << right_cam_info_topic);

    // Open ZED camera
    cv::Size2i image_size = cv::Size2i(mCamResWidth, mCamResHeight);
    std::string calibration_file;
    if (downloadCalibrationFile(mZedSN, calibration_file)) // Download camera calibration file
    {
        return false;
    }

    initCalibration(calibration_file, image_size, mCamLeftMapX, mCamLeftMapY, mCamRightMapX, mCamRightMapY, mCamMatrixLeftRect, mCamMatrixRightRect,
                    mCamMatrixLeftRaw, mCamMatrixRightRaw, mCamDistLeft, mCamDistRight);

    mVideoCap.open(mCamDeviceID);
    if (!mVideoCap.isOpened())
    {
        ROS_WARN_STREAM("Unable to open ZED camera with device id: " << mCamDeviceID);
        return false;
    }

    mVideoCap.set(CV_CAP_PROP_FRAME_WIDTH, image_size.width * 2);
    mVideoCap.set(CV_CAP_PROP_FRAME_HEIGHT, image_size.height);
    mVideoCap.grab();

    return true;
}

void ZedNativeHandler::Run()
{
    FillCamInfo();
    ros::Rate loop_rate(mCamFps);
    ros::Time time_stamp;
    cv::Mat frame, left_raw, left_rect, right_raw, right_rect;
    while (ros::ok())
    {
        time_stamp = ros::Time::now();

        mVideoCap >> frame;
        left_raw = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
        right_raw = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
        cv::remap(left_raw, left_rect, mCamLeftMapX, mCamLeftMapY, cv::INTER_LINEAR);
        cv::remap(right_raw, right_rect, mCamRightMapX, mCamRightMapY, cv::INTER_LINEAR);

        PublishCamInfo(mLeftCamInfoMsg, mPubLeftCamInfo, time_stamp);
        PublishCamInfo(mRightCamInfoMsg, mPubRightCamInfo, time_stamp);

        if (mPublishRaw)
        {
            PublishImage(left_raw, mPubRawLeft, mLeftCamOptFrameId, time_stamp);
            PublishImage(right_raw, mPubRawRight, mRightCamOptFrameId, time_stamp);
        }

        if (mPublishRectified)
        {
            PublishImage(left_rect, mPubLeft, mLeftCamOptFrameId, time_stamp);
            PublishImage(right_rect, mPubRight, mRightCamOptFrameId, time_stamp);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void ZedNativeHandler::FillCamInfo()
{
    sensor_msgs::CameraInfoPtr leftCamInfoMsg = mLeftCamInfoMsg;
    sensor_msgs::CameraInfoPtr rightCamInfoMsg = mRightCamInfoMsg;

    leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    leftCamInfoMsg->D.resize(5);
    rightCamInfoMsg->D.resize(5);
    leftCamInfoMsg->D[0] = mCamDistLeft.at<double>(0, 0);   // k1
    leftCamInfoMsg->D[1] = mCamDistLeft.at<double>(1, 0);   // k2
    leftCamInfoMsg->D[2] = mCamDistLeft.at<double>(4, 0);   // k3
    leftCamInfoMsg->D[3] = mCamDistLeft.at<double>(2, 0);   // p1
    leftCamInfoMsg->D[4] = mCamDistLeft.at<double>(3, 0);   // p2
    rightCamInfoMsg->D[0] = mCamDistRight.at<double>(0, 0); // k1
    rightCamInfoMsg->D[1] = mCamDistRight.at<double>(1, 0); // k2
    rightCamInfoMsg->D[2] = mCamDistRight.at<double>(4, 0); // k3
    rightCamInfoMsg->D[3] = mCamDistRight.at<double>(2, 0); // p1
    rightCamInfoMsg->D[4] = mCamDistRight.at<double>(3, 0); // p2

    leftCamInfoMsg->K.fill(0.0);
    rightCamInfoMsg->K.fill(0.0);
    leftCamInfoMsg->K[0] = mCamMatrixLeftRaw.at<double>(0, 0);
    leftCamInfoMsg->K[2] = mCamMatrixLeftRaw.at<double>(0, 2);
    leftCamInfoMsg->K[4] = mCamMatrixLeftRaw.at<double>(1, 1);
    leftCamInfoMsg->K[5] = mCamMatrixLeftRaw.at<double>(1, 2);
    leftCamInfoMsg->K[8] = 1.0;
    rightCamInfoMsg->K[0] = mCamMatrixRightRaw.at<double>(0, 0);
    rightCamInfoMsg->K[2] = mCamMatrixRightRaw.at<double>(0, 2);
    rightCamInfoMsg->K[4] = mCamMatrixRightRaw.at<double>(1, 1);
    rightCamInfoMsg->K[5] = mCamMatrixRightRaw.at<double>(1, 2);
    rightCamInfoMsg->K[8] = 1.0;

    leftCamInfoMsg->R.fill(0.0);
    rightCamInfoMsg->R.fill(0.0);
    for (size_t i = 0; i < 3; i++)
    {
        // This is currently set to identity, it must be fixed if the value is needed.
        rightCamInfoMsg->R[i + i * 3] = 1;
        leftCamInfoMsg->R[i + i * 3] = 1;
    }

    leftCamInfoMsg->P.fill(0.0);
    rightCamInfoMsg->P.fill(0.0);
    leftCamInfoMsg->P[0] = mCamMatrixLeftRect.at<double>(0, 0);
    leftCamInfoMsg->P[2] = mCamMatrixLeftRect.at<double>(0, 2);
    leftCamInfoMsg->P[5] = mCamMatrixLeftRect.at<double>(1, 1);
    leftCamInfoMsg->P[6] = mCamMatrixLeftRect.at<double>(1, 2);
    leftCamInfoMsg->P[10] = 1.0;
    const float baseline = 0.12; // in meters
    rightCamInfoMsg->P[3] = -mCamMatrixRightRect.at<double>(0, 0) * baseline;
    rightCamInfoMsg->P[0] = mCamMatrixRightRect.at<double>(0, 0);
    rightCamInfoMsg->P[2] = mCamMatrixRightRect.at<double>(0, 2);
    rightCamInfoMsg->P[5] = mCamMatrixRightRect.at<double>(1, 1);
    rightCamInfoMsg->P[6] = mCamMatrixRightRect.at<double>(1, 2);
    rightCamInfoMsg->P[10] = 1.0;
    leftCamInfoMsg->width = rightCamInfoMsg->width = static_cast<uint32_t>(mCamResWidth);
    leftCamInfoMsg->height = rightCamInfoMsg->height = static_cast<uint32_t>(mCamResHeight);
    leftCamInfoMsg->header.frame_id = mLeftCamOptFrameId;
    rightCamInfoMsg->header.frame_id = mRightCamOptFrameId;
}

void ZedNativeHandler::PublishCamInfo(sensor_msgs::CameraInfoPtr camInfoMsg, ros::Publisher pubCamInfo, ros::Time t)
{
    camInfoMsg->header.stamp = t;
    pubCamInfo.publish(camInfoMsg);
}

void ZedNativeHandler::PublishImage(cv::Mat img, image_transport::Publisher &pubImg, std::string imgFrameId, ros::Time t)
{
    sensor_msgs::ImagePtr msg;
    if (mPublishGray)
    {
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    }
    else
    {
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    }
    msg->header.stamp = t;
    msg->header.frame_id = imgFrameId;
    pubImg.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_opencv_ros");
    ZedNativeHandler h;
    if (!h.Init())
    {
        return -1;
    }
    h.Run();
    return 0;
}
