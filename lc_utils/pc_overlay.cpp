// Author: Joe


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <ros/package.h>
#include <ros/xmlrpc_manager.h>
#include <signal.h>
#include "Eigen/Core"
#include "Eigen/Geometry"
//#include <dynamic_reconfigure/server.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include "opencv2/xphoto.hpp"
#include <opencv2/core/eigen.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <chrono>


image_transport::Publisher image_overlay_pub; //raw 16-bit data (get from the curtain object)
std::string pcld_topic = "/redversion/cloud/all";
std::string img_topic = "/redversion//narrow_stereo/image_color_throttle";
std::string info_topic = "/redversion/narrow_stereo/camera_info";
std::string overlay_topic = "/redversion/overlay/image_color";


ros::Subscriber pointcloud_sub;
ros::Subscriber camimg_sub;

int k_size = 3;

int imgNum=0;
int cldNum=0;

tf::TransformListener *listener;
image_geometry::PinholeCameraModel model_;


std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
std::chrono::steady_clock::time_point tbegin = std::chrono::steady_clock::now();
std::chrono::steady_clock::time_point tend = std::chrono::steady_clock::now();

using namespace std;

// typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> MySyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> MySyncPolicy;

cv::Mat rotationFromTransform(const tf::Transform &t) {
    return (cv::Mat_<double>(3, 3) << t.getBasis()[0][0], t.getBasis()[0][1], t.getBasis()[0][2],
            t.getBasis()[1][0], t.getBasis()[1][1], t.getBasis()[1][2],
            t.getBasis()[2][0], t.getBasis()[2][1], t.getBasis()[2][2]);
}

cv::Mat translationFromTransform(const tf::Transform &t) {
    return (cv::Mat_<double>(3, 1) << t.getOrigin()[0], t.getOrigin()[1], t.getOrigin()[2]);
}

void callback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg, const sensor_msgs::PointCloud2ConstPtr& cld_msg)
{
   // std::cout << "img_stamp: " << img_msg->header.stamp << std::endl;
   // std::cout << "info_stamp: " << info_msg->header.stamp << std::endl;
   // std::cout << "cld_stamp: " << cld_msg->header.stamp << std::endl;

   // std::cout << "tdif: " << cld_msg->header.stamp - img_msg->header.stamp << std::endl;

    ros::Time pub_time = cld_msg->header.stamp;

    cv::Mat rect;

// If zero distortion, just pass the message along
    bool zero_distortion = true;
    for(size_t i = 0; i < info_msg->D.size(); ++i)
    {
        if (info_msg->D[i] != 0.0)
        {
            zero_distortion = false;
            break;
        }
    }
    // This will be true if D is not empty/zero sized
    if(!zero_distortion) {
//        ROS_INFO("DISTORTION MODEL LOADED!");
        // Update the camera model
        model_.fromCameraInfo(info_msg);

        // Create cv::Mat views onto both buffers
        const cv::Mat image = cv_bridge::toCvShare(img_msg)->image;

        // Rectify
        model_.rectifyImage(image, rect, CV_INTER_CUBIC);
    }
    else //no distortion, so use as is
        rect = cv_bridge::toCvShare(img_msg)->image;

    if(rect.channels()==1) //convert to color image if mono
        cvtColor(rect, rect, CV_GRAY2BGR);

//    wb->balanceWhite(rect, rect);
//
//    rect = 0.7*rect; //brighten image

    cv::Matx33d K_cv = model_.fullIntrinsicMatrix();

    Eigen::Matrix<float, 3, 4> K34; //pad k matrix to make 4x4
    K34 <<  K_cv(0,0), K_cv(0,1), K_cv(0,2), 0,
            K_cv(1,0), K_cv(1,1), K_cv(1,2), 0,
            K_cv(2,0), K_cv(2,1), K_cv(2,2), 0;
           // 0, 0, 0, 1;

//    std::cout << "Camera Matrix: " << K44 << std::endl;

    tf::StampedTransform sTf;
    try{
        // listener->waitForTransform(img_msg->header.frame_id, cld_msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
        listener->lookupTransform("red_camera2", cld_msg->header.frame_id, ros::Time(0), sTf);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("OVERLAY ERROR %s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // cv::Mat rand_mask(rect.size(), CV_8UC1);
    // std::cout << "size of the rand mask is: " << rand_mask.size() << std::endl;
    // cv::randu(rand_mask, 0, 255);
    // rand_mask.setTo(255, rand_mask>127);
    // std::cout << "setTo is the problem" << std::endl;
    // std::cout << "random thing is the problem" << std::endl;


    //Eigen::Matrix4f P = K44*tf_eig; //projection matrix to transform points

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cld_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cld_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cld_msg, "z");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_rgb(*cld_msg, "rgb");

    if(0) { //slow way?
        Eigen::Matrix4f tf_eig;

        tf::Transform tform(sTf.getBasis(),sTf.getOrigin());

        pcl_ros::transformAsMatrix(tform, tf_eig);

        cv::Mat cld_img(rect.size(), CV_8UC3, cv::Scalar(0,0,0));

        for (int v = 0; v < int(cld_msg->height); ++v) {
            for (int u = 0; u < int(cld_msg->width); ++u, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
                if (std::isfinite(*iter_x) && std::isfinite(*iter_y) &&
                    std::isfinite(*iter_z)) { //only project point if it is not nan or inf
                    Eigen::Vector4f pt(*iter_x, *iter_y, *iter_z, 1);
                    Eigen::Vector4f new_pt = tf_eig * pt; //transform point in original frame to new frame
                    //Eigen::Vector4f pt_out = P * pt; //project point to image plane to get [u' v' q'] ==> divide by q' to get [u v 1]
                    Eigen::Vector3f new_coords = K34 * new_pt;

                    //get image pt
                    Eigen::Vector3f image_pt = new_coords.head(3) / new_coords(2); // ==> divide by q' to get [u v 1]

                    //color image with color of pointcloud
                    int u_img = round(image_pt(0));
                    int v_img = round(image_pt(1));
//            std::cout << "[u,v] = [" << u_img << "," << v_img << "]" << std::endl;
                    if ((u_img >= k_size / 2) && (u_img < (rect.cols - k_size / 2)) && (v_img >= k_size / 2 ) && (v_img < (rect.rows -k_size /2)) && ((u_img%4))==0) //if projected point is within image bounds color that pixel the color of the point cloud point
                    {
                        cv::Rect roi(cv::Point(u_img - k_size / 2, v_img - k_size / 2), cv::Size(k_size, k_size));
                        rect(roi).setTo(cv::Scalar(iter_rgb[0], iter_rgb[1],
                                                   iter_rgb[2])); //color k_size x k_size region around the point to make curtain more visible in overlay image
                    }
                }
            }
        }
    }

    if(0) {
        Eigen::Matrix4f tf_eig;

        tf::Transform tform(sTf.getBasis(),sTf.getOrigin());

        pcl_ros::transformAsMatrix(tform, tf_eig);

        cv::Mat cld_img(rect.size(), CV_8UC3, cv::Scalar(0,0,0));
        cv::Mat cld_img_mask(rect.size(), CV_8UC1, cv::Scalar(0));

        cv::Mat comb_mask(rect.size(), CV_8UC1, cv::Scalar(0));
        cv::Mat overlay(rect.size(), CV_8UC3, cv::Scalar(0,0,0));

        cv::Mat sparse_mask(rect.size(), CV_8UC1,cv::Scalar(0));

        for (int i=3; i<rect.cols; i+=5)
        {
            if( (i-1) >= 0 )
                sparse_mask.col(i-1).setTo(cv::Scalar(1));

            sparse_mask.col(i).setTo(cv::Scalar(1));

            if((i+1)<rect.cols)
                sparse_mask.col(i+1).setTo(cv::Scalar(1));

        }

        for (int v = 0; v < int(cld_msg->height); ++v) {
            for (int u = 0; u < int(cld_msg->width); ++u, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
                if (std::isfinite(*iter_x) && std::isfinite(*iter_y) &&
                    std::isfinite(*iter_z)) { //only project point if it is not nan or inf
                    Eigen::Vector4f pt(*iter_x, *iter_y, *iter_z, 1);
                    Eigen::Vector4f new_pt = tf_eig * pt; //transform point in original frame to new frame
                    //Eigen::Vector4f pt_out = P * pt; //project point to image plane to get [u' v' q'] ==> divide by q' to get [u v 1]
                    Eigen::Vector3f new_coords = K34 * new_pt;

                    //get image pt
                    Eigen::Vector3f image_pt = new_coords.head(3) / new_coords(2); // ==> divide by q' to get [u v 1]

                    //color image with color of pointcloud
                    int u_img = round(image_pt(0));
                    int v_img = round(image_pt(1));
//            std::cout << "[u,v] = [" << u_img << "," << v_img << "]" << std::endl;
                    if ((u_img >= k_size / 2) && (u_img < (rect.cols - k_size / 2)) && (v_img >= k_size / 2 ) && (v_img < (rect.rows -k_size /2))) //if projected point is within image bounds color that pixel the color of the point cloud point
                    {
                        cv::Rect roi(cv::Point(u_img - k_size / 2, v_img - k_size / 2), cv::Size(k_size, k_size));
                        cld_img(roi).setTo(cv::Scalar(iter_rgb[0], iter_rgb[1],
                                                   iter_rgb[2])); //color k_size x k_size region around the point to make curtain more visible in overlay image
                        cld_img_mask(roi).setTo(cv::Scalar(255));
                    }
                }
            }
        }

        bitwise_and(sparse_mask, cld_img_mask, comb_mask);

        cld_img.copyTo(rect,comb_mask);
    }

    if(1) {
        tbegin = std::chrono::steady_clock::now();
        t0 = std::chrono::steady_clock::now();


        std::vector<cv::Point3f> obj_pts;//(cld_msg->height*cld_msg->width);
        std::vector<cv::Point2f> img_pts;//(cld_msg->height*cld_msg->width);
        std::vector<cv::Point3i> rgb_pts;//(cld_msg->height*cld_msg->width);

        cv::Mat camRotation = rotationFromTransform(sTf);
        cv::Mat camTranslation = translationFromTransform(sTf);

//         cv::Mat cld_img(rect.size(), CV_8UC3, cv::Scalar(0,0,0));
//         cv::Mat cld_img_mask(rect.size(), CV_8UC1, cv::Scalar(0));

//         cv::Mat comb_mask(rect.size(), CV_8UC1, cv::Scalar(0));
        // cv::Mat overlay(rect.size(), CV_8UC3, cv::Scalar(0,0,0));

        // cv::Mat sparse_mask(rect.size(), CV_8UC1,cv::Scalar(0));

//         std::cout << "setup time = " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tbegin).count() << "[ms]" << std::endl;
//         tbegin = std::chrono::steady_clock::now();

        // for (int i=3; i<rect.cols; i+=5)
        // {
        //     if( (i-1) >= 0 )
        //         sparse_mask.col(i-1).setTo(cv::Scalar(1));

        //     sparse_mask.col(i).setTo(cv::Scalar(1));

        //     if((i+1)<rect.cols)
        //         sparse_mask.col(i+1).setTo(cv::Scalar(1));

        // }

        // std::cout << "sparse time = " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tbegin).count() << "[ms]" << std::endl;
        // tbegin = std::chrono::steady_clock::now();

        for (int v = 0; v < int(cld_msg->height); ++v) {
            for (int u = 0; u < int(cld_msg->width); ++u, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
                if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z) && (u % 2) == 0 &&
                    (v % 2) == 0) { //only project point if it is not nan or inf

                    obj_pts.push_back(cv::Point3f(*iter_x, *iter_y, *iter_z));
                    rgb_pts.push_back(cv::Point3i(iter_rgb[0], iter_rgb[1], iter_rgb[2]));
                }
            }
        }

        // std::cout << "obj points shape: " << obj_pts.size() << std::endl;
        // std::cout << "rgb points shape: " << rgb_pts.size() << std::endl;

        // std::cout << "pts time = " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tbegin).count() << "[ms]" << std::endl;
        // tbegin = std::chrono::steady_clock::now();



        //project points to image using opencv
        if (!obj_pts.empty()) {
//            cv::projectPoints(obj_pts, camRotation, camTranslation, model_.fullIntrinsicMatrix(),
//                              model_.distortionCoeffs(), img_pts); //only use if projecting onto a distorted image
            cv::projectPoints(obj_pts, camRotation, camTranslation, model_.fullIntrinsicMatrix(), cv::Mat(), img_pts);

            // std::cout << "proj time = " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tbegin).count() << "[ms]" << std::endl;
            // tbegin = std::chrono::steady_clock::now();

            for (size_t i = 0; i < obj_pts.size(); i++) {

                //color image with color of pointcloud
                int u_img = round(img_pts[i].x);
                int v_img = round(img_pts[i].y);
//            std::cout << "[u,v] = [" << u_img << "," << v_img << "]" << std::endl;
                if (u_img >= 0 && u_img < (rect.cols) && v_img >= k_size / 2 && v_img <
                                                                                (rect.rows)) //if projected point is within image bounds color that pixel the color of the point cloud point
                {

                    // cv::Rect roi(cv::Point(u_img - k_size / 2, v_img - k_size / 2), cv::Size(k_size, k_size));
                    // rect(roi).setTo(cv::Scalar(rgb_pts[i].x,rgb_pts[i].y,rgb_pts[i].z)); //color k_size x k_size region around the point to make curtain more visible in overlay image
                    // cld_img(roi).setTo(cv::Scalar(rgb_pts[i].x,rgb_pts[i].y,rgb_pts[i].z)); //color k_size x k_size region around the point to make curtain more visible in overlay image
                    // cld_img_mask(roi).setTo(cv::Scalar(255));

                    //set pixels around point to same color as point (make it look bigger)
                    for (int j = -1; j < 2; j++) {
                        if (j + v_img > 0 && j + v_img < rect.rows) {
                            for (int k = -1; k < 2; k++) {
                                if (k + u_img > 0 && k + u_img < rect.cols) {
                                    rect.at<cv::Vec3b>(v_img + j, u_img + k) = cv::Vec3b(rgb_pts[i].x, rgb_pts[i].y,
                                                                                         rgb_pts[i].z); //fastest (only line used)
                                }
                            }
                        }
                    }
//                    rect.at<cv::Vec3b>(v_img, u_img) = cv::Vec3b(rgb_pts[i].x, rgb_pts[i].y, rgb_pts[i].z); //fastest (only line used)

                    // cld_img.at<cv::Vec3b>(v_img,u_img) = cv::Vec3b(rgb_pts[i].x,rgb_pts[i].y,rgb_pts[i].z);
//                cld_img_mask.at<uchar>(v_img, u_img) = 255;
                }
            }

            // std::cout << "copy time = " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tbegin).count() << "[ms]" << std::endl;
            // tbegin = std::chrono::steady_clock::now();

            // bitwise_and(rand_mask, cld_img_mask, comb_mask);
            // cld_img.copyTo(rect,cld_img_mask);

            // std::cout << "end time = " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tbegin).count() << "[ms]" << std::endl;
//        std::cout << "total time = " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count() << "[ms]" << std::endl;
//        tbegin = std::chrono::steady_clock::now();

            //publish message
            // cout << "done with the processing publishing image now" << endl;
            sensor_msgs::ImagePtr out_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rect).toImageMsg();
            out_img->header.stamp = pub_time;
            out_img->header.frame_id = img_msg->header.frame_id;
            image_overlay_pub.publish(out_img);
        }
    }


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_overlay");
    ros::NodeHandle nh;

    cout << "pcld topic: " << pcld_topic << endl;
    cout << "img topic: " << img_topic << endl;
    cout << "info topic: " << info_topic << endl;
    cout << "overlay_topic" << overlay_topic << endl;

    image_transport::ImageTransport it_overlay(nh);
    image_overlay_pub = it_overlay.advertise(overlay_topic,1);

    image_transport::ImageTransport it_sub(nh);
    image_transport::SubscriberFilter img_sub;
    img_sub.subscribe(it_sub, img_topic,10);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh,info_topic,10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcld_sub(nh, pcld_topic,10);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img_sub, info_sub, pcld_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));
    std::cout << "listening.." << std::endl;

    listener = new tf::TransformListener();

    ros::Rate loop_rate(60); //operate at 120hz
    while (nh.ok()) {
        ros::spin();
        // loop_rate.sleep(); //commented out to make it run as fast as possible
    }
    ROS_INFO("Shutting Down!");
    ros::shutdown();
    return 0;
}
