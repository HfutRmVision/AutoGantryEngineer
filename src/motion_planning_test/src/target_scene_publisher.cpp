/**
 * @file target_scene_publisher.cpp
 * @brief 发布 planning_scene，固定发布评审位姿 3.1 的 cylinder target
 *
 * 固定位姿 3.1（相对于 ipm）:
 * - 位置: x=75mm, y=0mm, z=-24mm
 * - 姿态: roll=-50°, pitch=-90°, yaw=45°
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <array>
#include <vector>
#include <random>

class TargetScenePublisher : public rclcpp::Node
{
public:
    TargetScenePublisher()
        : Node("target_scene_publisher"),
          random_device_(),
          random_engine_(random_device_()),
          dist_x_(-0.100, 0.0),
          dist_y_(0.100, 0.300),
          dist_z_(0.500, 0.700),
          dist_roll_deg_(-0.0, 0.0),
          dist_pitch_deg_(-0.0, 0.0),
          dist_yaw_deg_(-0.0, 0.0)
    {
        // 声明参数
        this->declare_parameter("reference_frame", "ipm");
        this->declare_parameter("target_name", "cylinder_target");
        this->declare_parameter("cylinder_radius", 0.018);
        this->declare_parameter("cylinder_length", 0.1);
        
        // 获取参数
        reference_frame_ = this->get_parameter("reference_frame").as_string();
        target_name_ = this->get_parameter("target_name").as_string();
        cylinder_radius_ = this->get_parameter("cylinder_radius").as_double();
        cylinder_length_ = this->get_parameter("cylinder_length").as_double();

        // 评审位姿表: {x(mm), y(mm), z(mm), roll(deg), pitch(deg), yaw(deg)}
        evaluation_poses_ = {
            {0.0,   -50.0,  -25.0, -90.0,   0.0,   0.0},   // 1.1
            {0.0,   -50.0,  -70.0, -90.0,   0.0,   0.0},   // 1.2
            {0.0,   -25.0,  -50.0, -30.0, -20.0,  20.0},   // 2.1
            {0.0,  -100.0,    0.0, -90.0, -45.0,  20.0},   // 2.2
            {275.0, 600.0,  -24.0, -40.0, -90.0,  45.0},   // 3.1
            {160.0, 500.0,  -48.0, -50.0,  90.0, -45.0}    // 3.2
        };

        
        // TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // Planning scene publisher
        planning_scene_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
            "planning_scene", 10);
        
        // 服务：手动触发重新发布随机位姿
        generate_target_service_ = this->create_service<std_srvs::srv::Trigger>(
            "publish_target_3_1",
            std::bind(&TargetScenePublisher::generateTargetCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Target Scene Publisher 初始化完成");
        RCLCPP_INFO(this->get_logger(), "参考坐标系: %s", reference_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Cylinder: 半径=%.3fm, 长度=%.3fm", cylinder_radius_, cylinder_length_);
        RCLCPP_INFO(this->get_logger(), "发布模式: 随机发布位姿");
        RCLCPP_INFO(this->get_logger(),
                "随机范围: x(-100,0)mm, y(100,300)mm, z(500,700)mm, roll(-45,45)deg, pitch(-90,0)deg, yaw(-90,90)deg");
        
        // 延迟初始化，等待 MoveIt 和 TF 准备好
        init_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&TargetScenePublisher::initCallback, this));
    }

private:
    void initCallback()
    {
        init_timer_->cancel();

        // 发布随机位姿
        publishRandomTarget();
        
        // 启动 TF 发布定时器，每 50ms 发布一次（20Hz）
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TargetScenePublisher::publishTargetTFTimer, this));
    }

    geometry_msgs::msg::Pose buildPoseFromMetersAndDegrees(
        double x_m,
        double y_m,
        double z_m,
        double roll_deg,
        double pitch_deg,
        double yaw_deg) const
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x_m;
        pose.position.y = y_m;
        pose.position.z = z_m;

        const double roll = roll_deg * M_PI / 180.0;
        const double pitch = pitch_deg * M_PI / 180.0;
        const double yaw = yaw_deg * M_PI / 180.0;

        tf2::Quaternion q_roll, q_pitch, q_yaw;
        q_roll.setRPY(roll, 0.0, 0.0);
        q_pitch.setRPY(0.0, pitch, 0.0);
        q_yaw.setRPY(0.0, 0.0, yaw);

        tf2::Quaternion q = q_pitch * q_roll * q_yaw;
        q.normalize();
        pose.orientation = tf2::toMsg(q);

        return pose;
    }

    geometry_msgs::msg::Pose buildPoseFromEvaluation(const std::array<double, 6>& pose6) const
    {
        geometry_msgs::msg::Pose pose;

        // mm -> m
        pose.position.x = pose6[0] / 1000.0;
        pose.position.y = pose6[1] / 1000.0;
        pose.position.z = pose6[2] / 1000.0;

        // deg -> rad
        const double roll = pose6[3] * M_PI / 180.0;
        const double pitch = pose6[4] * M_PI / 180.0;
        const double yaw = pose6[5] * M_PI / 180.0;
        
        // 1. 声明并生成三个纯净的单轴旋转（此时它们都严格参考 ipm）
        tf2::Quaternion q_roll, q_pitch, q_yaw;
        q_roll.setRPY(roll, 0.0, 0.0);   // 绕 ipm 的 X 轴
        q_pitch.setRPY(0.0, pitch, 0.0); // 绕 ipm 的 Y 轴
        q_yaw.setRPY(0.0, 0.0, yaw);     // 绕 ipm 的 Z 轴

        // 2. 绕固定参考系 (ipm) 依次旋转：先 Yaw(Z) -> 再 Roll(X) -> 最后 Pitch(Y)
        // 左乘法则： q = (最后一步 Roll) * (第二步 Pitch) * (第一步 Yaw)
        tf2::Quaternion q = q_roll * q_pitch * q_yaw;
        
        q.normalize();
        pose.orientation = tf2::toMsg(q);

        return pose;
    }
    
    moveit_msgs::msg::CollisionObject createCylinderTarget(const geometry_msgs::msg::Pose& pose)
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = reference_frame_;
        collision_object.header.stamp = this->now();
        collision_object.id = target_name_;
        
        // 定义 cylinder 形状
        // 注意: MoveIt 的 cylinder 默认原点在中心，我们需要将其偏移使原点在底面中心
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[primitive.CYLINDER_HEIGHT] = cylinder_length_;
        primitive.dimensions[primitive.CYLINDER_RADIUS] = cylinder_radius_;
        
        // 调整位姿，使得坐标原点在底面中心
        // cylinder 默认原点在中心，需要沿 z 轴（cylinder 轴向）偏移 length/2
        geometry_msgs::msg::Pose adjusted_pose = pose;
        
        // 获取当前姿态的旋转矩阵
        tf2::Quaternion q;
        tf2::fromMsg(pose.orientation, q);
        tf2::Matrix3x3 rotation_matrix(q);
        
        // cylinder 在其局部坐标系中沿 z 轴，需要将中心偏移到底面
        // 偏移量为 -length/2，使底面在原点，z轴正方向从底面向外指
        tf2::Vector3 offset(0, 0, -cylinder_length_ / 2.0);
        tf2::Vector3 world_offset = rotation_matrix * offset;
        
        adjusted_pose.position.x += world_offset.x();
        adjusted_pose.position.y += world_offset.y();
        adjusted_pose.position.z += world_offset.z();
        
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(adjusted_pose);
        collision_object.operation = collision_object.ADD;
        
        return collision_object;
    }
    
    void publishRandomTarget()
    {
        const double x_m = dist_x_(random_engine_);
        const double y_m = dist_y_(random_engine_);
        const double z_m = dist_z_(random_engine_);
        const double roll_deg = dist_roll_deg_(random_engine_);
        const double pitch_deg = dist_pitch_deg_(random_engine_);
        const double yaw_deg = dist_yaw_deg_(random_engine_);

        geometry_msgs::msg::Pose random_pose =
            buildPoseFromMetersAndDegrees(x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg);

        RCLCPP_INFO(this->get_logger(),
                    "发布随机位姿: mm(%.1f, %.1f, %.1f), deg(%.1f, %.1f, %.1f)",
                    x_m * 1000.0, y_m * 1000.0, z_m * 1000.0,
                    roll_deg, pitch_deg, yaw_deg);
        publishTargetByPose(random_pose, "随机位姿");
    }

    void publishTargetByPose(const geometry_msgs::msg::Pose& pose, const std::string& source)
    {
        // 创建 collision object
        moveit_msgs::msg::CollisionObject target = createCylinderTarget(pose);
        
        // 创建 planning scene 消息
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        planning_scene_msg.world.collision_objects.push_back(target);
        
        // 发布
        planning_scene_pub_->publish(planning_scene_msg);
        
        RCLCPP_INFO(this->get_logger(), "已发布 %s cylinder target '%s' 到 planning scene", 
                source.c_str(), target_name_.c_str());
        
        // 保存当前 target 位姿（相对于参考系的原始位姿，底面中心）
        current_target_pose_ = pose;
        
        // 发布 TF 以显示坐标轴
        publishTargetTF(pose);
    }
    
    void publishTargetTF(const geometry_msgs::msg::Pose& pose)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = reference_frame_;
        transform.child_frame_id = target_name_ + "_frame";
        
        transform.transform.translation.x = pose.position.x;
        transform.transform.translation.y = pose.position.y;
        transform.transform.translation.z = pose.position.z;

        // 圆柱几何仍使用局部 Z 轴作为轴向；仅对发布的 target frame 做轴重映射：
        // 让 "底面法线方向" 在 target frame 中对应 X 轴（原来对应 Z 轴）。
        tf2::Quaternion q_pose;
        tf2::fromMsg(pose.orientation, q_pose);

        tf2::Quaternion q_axis_remap;
        q_axis_remap.setRPY(0.0, -M_PI / 2.0, 0.0);  // local X -> old local Z

        tf2::Quaternion q_tf = q_pose * q_axis_remap;
        q_tf.normalize();
        transform.transform.rotation = tf2::toMsg(q_tf);
        
        tf_broadcaster_->sendTransform(transform);
    }
    
    void publishTargetTFTimer()
    {
        // 定时器回调，持续发布当前 target 的 TF
        publishTargetTF(current_target_pose_);
    }
    
    void generateTargetCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        publishRandomTarget();
        response->message = "已重新发布随机位姿";
        response->success = true;
    }
    
    void removeTarget()
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = reference_frame_;
        collision_object.header.stamp = this->now();
        collision_object.id = target_name_;
        collision_object.operation = collision_object.REMOVE;
        
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        planning_scene_msg.world.collision_objects.push_back(collision_object);
        
        planning_scene_pub_->publish(planning_scene_msg);
        
        RCLCPP_INFO(this->get_logger(), "已从 planning scene 中移除 '%s'", target_name_.c_str());
    }

    // 参数
    std::string reference_frame_;
    std::string target_name_;
    double cylinder_radius_;
    double cylinder_length_;
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Publishers
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
    
    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr generate_target_service_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    
    // 当前 target 位姿
    geometry_msgs::msg::Pose current_target_pose_;

    // 评审位姿表（保留，固定使用 3.1）
    std::vector<std::array<double, 6>> evaluation_poses_;
    const size_t fixed_evaluation_index_ = 5;  // 3.2

    // 随机位姿生成器
    std::random_device random_device_;
    std::mt19937 random_engine_;
    std::uniform_real_distribution<double> dist_x_;
    std::uniform_real_distribution<double> dist_y_;
    std::uniform_real_distribution<double> dist_z_;
    std::uniform_real_distribution<double> dist_roll_deg_;
    std::uniform_real_distribution<double> dist_pitch_deg_;
    std::uniform_real_distribution<double> dist_yaw_deg_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetScenePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
