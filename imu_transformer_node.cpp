#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher imu_pub; // パブリッシャーをグローバルスコープで宣言

void callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    // カメラ座標系からcamera_link座標系への変換を取得
    geometry_msgs::TransformStamped transform;
    transform = tf_buffer.lookupTransform("camera_link", "camera_imu_optical_frame", ros::Time(0));

    // IMUデータのorientation部分にトランスフォームを適用
    tf2::doTransform(imu_msg.orientation, imu_msg.orientation, transform);

    imu_pub.publish(imu_msg)
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_transformer");
    ros::NodeHandle nh;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    imu_pub = nh.advertise<sensor_msgs::Imu>("/camera_link/imu", 10); // パブリッシャーをグローバルスコープで初期化
    ros::Subscriber imu_sub = nh.subscribe("/camera/imu", 10, callback);

    ros::spin();
    return 0;
}
