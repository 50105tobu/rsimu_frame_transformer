#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_transformer");
    ros::NodeHandle nh;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/camera_link/imu", 10); // 新しいimuトピックをパブリッシュ

    ros::Rate rate(10.0); // ループの周期(Hz)

    while (ros::ok())
    {
        sensor_msgs::Imu imu_data;
        try
        {
            // IMUデータを購読
            sensor_msgs::Imu imu_msg;
            imu_msg = ros::topic::waitForMessage<sensor_msgs::Imu>("/camera/imu", nh);

            // カメラ座標系からcamera_link座標系への変換を取得
            geometry_msgs::TransformStamped transform;
            transform = tf_buffer.lookupTransform("camera_link", "camera_imu_optical_frame", ros::Time(0));

            // IMUデータのコピー
            imu_data = imu_msg;

            // IMUデータのorientation部分にトランスフォームを適用
            tf2::doTransform(imu_data.orientation, imu_data.orientation, transform);

            // 新しいimuデータをパブリッシュ
            imu_pub.publish(imu_data);
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
        }

        rate.sleep();
    }

    return 0;
}
