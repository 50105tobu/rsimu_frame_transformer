#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher imu_pub; // publisherをグローバルスコープで宣言

void callback(const sensor_msgs::Imu::ConstPtr& imu_msg, tf2_ros::Buffer& tf_buffer)
{
    try
    {
        // カメラ座標系からcamera_link座標系への変換を取得
        geometry_msgs::TransformStamped transform;
        transform = tf_buffer.lookupTransform("camera_link", "camera_imu_optical_frame", imu_msg->header.stamp);
        //transform = tf_buffer.lookupTransform("camera_imu_optical_frame", "camera_link", imu_msg->header.stamp);

        // IMUデータのorientation部分にトランスフォームを適用
        geometry_msgs::Quaternion imu_orientation_transformed;
        geometry_msgs::Vector3 imu_angular_velocity_transformed;
        geometry_msgs::Vector3 imu_linear_acceleration_transformed;
        tf2::doTransform(imu_msg->orientation, imu_orientation_transformed, transform);
        tf2::doTransform(imu_msg->angular_velocity, imu_angular_velocity_transformed, transform);
        tf2::doTransform(imu_msg->linear_acceleration, imu_linear_acceleration_transformed, transform);

        // IMUメッセージをコピー
        sensor_msgs::Imu imu_msg_transformed = *imu_msg;
        
        // 変更箇所を代入
        imu_msg_transformed.orientation = imu_orientation_transformed;
        imu_msg_transformed.angular_velocity = imu_angular_velocity_transformed;
        imu_msg_transformed.linear_acceleration = imu_linear_acceleration_transformed;
        imu_msg_transformed.header.frame_id = "camera_link";

        imu_pub.publish(imu_msg_transformed);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("TF変換エラー: %s", ex.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_transformer_node");
    ros::NodeHandle nh;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    imu_pub = nh.advertise<sensor_msgs::Imu>("/camera_link/imu", 10); //パブリッシャーの設定
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/camera/imu", 10, boost::bind(callback, _1, boost::ref(tf_buffer)));

    ros::spin();
    return 0;
}
