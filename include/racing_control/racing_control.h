#ifndef RACING_CONTROL_H_
#define RACING_CONTROL_H_

#include <vector>
#include <optional>
#include <queue>
#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "std_msgs/msg/int32.hpp"

enum class State {
  LINE_FOLLOWING,       // 巡线模式
  OBSTACLE_AVOIDING,    // 避障模式
  RECOVERING_LINE,      // 避障后寻找赛道模式
  STOP,                 // 停止或等待
  PARKED                // 已完成停车
};

// 比较函数，用于 priority_queue 以时间戳升序排序 (即最新消息在top)
// 默认 priority_queue 是最大堆，所以我们要实现一个“小于”的比较器
// 这样最新的时间戳被认为是“最小”的，从而在 pop 时按正确顺序
struct compare_timestamps {
  bool operator()(const ai_msgs::msg::PerceptionTargets::SharedPtr m1,
                  const ai_msgs::msg::PerceptionTargets::SharedPtr m2) const {
    return (m1->header.stamp.sec < m2->header.stamp.sec) ||
           ((m1->header.stamp.sec == m2->header.stamp.sec) &&
            (m1->header.stamp.nanosec < m2->header.stamp.nanosec));
  }
};

class RacingControlNode : public rclcpp::Node{
public:
    RacingControlNode(const std::string& node_name,
                        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~RacingControlNode() override;
private:
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr point_subscriber_;
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr target_subscriber_;
    // 订阅 /sign4return 话题的订阅器
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sign_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    void subscription_callback_point(const ai_msgs::msg::PerceptionTargets::SharedPtr point_msg);
    void subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg);
    // 处理 /sign4return 消息的回调函数
    void sign_callback(const std_msgs::msg::Int32::SharedPtr msg);
    bool LineFollowing(float target_linear_speed);
    void ObstaclesAvoiding(const ai_msgs::msg::Target &obstacle_target);
    void MessageProcess(void);
    bool hasValidTrackLine(const ai_msgs::msg::PerceptionTargets::SharedPtr& line_msg) const; //检查赛道线消息是否有效且置信度足够高
    std::optional<ai_msgs::msg::Target> findFollowableParkingSign(const ai_msgs::msg::PerceptionTargets::SharedPtr& targets_msg) const; //查找一个置信度足够高、适合用于跟随的停车标志
    bool isParkingConditionMet(const ai_msgs::msg::PerceptionTargets::SharedPtr& targets_msg) const;  //检查是否满足最终的停车条件（即停车标志足够近）
    std::optional<ai_msgs::msg::Target> findAvoidanceObstacle(const ai_msgs::msg::PerceptionTargets::SharedPtr& targets_msg) const; //查找一个距离足够近、需要立即执行避障的障碍物
    bool isObstacleInCautionZone(const ai_msgs::msg::PerceptionTargets::SharedPtr& targets_msg) const;  //检查是否有障碍物进入了需要减速的“谨慎区域”
    bool isObstacleOnRecoverySide(const ai_msgs::msg::Target& obstacle) const; // 检查障碍物是否在回线方向的同侧
    
    std::string pub_control_topic_ = "cmd_vel";
    std::mutex point_target_mutex_;
    bool process_stop_ = false;
    std::shared_ptr<std::thread> msg_process_;
    
    // 巡线参数
    float follow_linear_speed_ = 0.5;
    float follow_angular_ratio_ = -0.8;
    float line_confidence_threshold_ = 0.9;
    
    // 避障参数
    float avoid_linear_speed_ = 0.3;
    float avoid_angular_ratio_ = 1.2;
    float obstacle_confidence_threshold_ = 0.8;
    int bottom_threshold_caution_ = 280; 
    int bottom_threshold_avoid_ = 320; 
    int parking_y_threshold_ = 300;
    
    // 低置信度巡航和寻找赛道时的参数
    float parking_sign_confidence_threshold_ = 0.3;
    float cruise_linear_speed_ = 0.2;       // 低置信度时慢速直行的速度
    float recovering_linear_speed_ = 0.7;   // 寻找赛道时的线速度
    float recovering_angular_ratio_ = 0.8;  // 寻找赛道时的角速度比例

    // 状态机变量
    State current_state_ = State::LINE_FOLLOWING;
    float last_avoidance_angular_z_ = 0.0;  // 记录最后一次避障的角速度
 
    float last_avoidance_direction_ = 0.0;  // 记录最后一次避障的方向(符号和last_avoidance_angular_z_相同，值取-1或1)

    geometry_msgs::msg::Twist last_valid_twist_;
    bool has_valid_twist_ = false;          // 标志位，确保我们有一个可用的指令

    // 最新消息存储
    ai_msgs::msg::PerceptionTargets::SharedPtr latest_point_msg_;
    ai_msgs::msg::PerceptionTargets::SharedPtr latest_targets_msg_;

    // 挂起/恢复状态标志
    std::atomic<bool> is_suspended_{false};
};

#endif  // RACING_CONTROL_H_
