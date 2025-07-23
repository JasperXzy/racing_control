#include "racing_control/racing_control.h"
#include <unistd.h>
#include <chrono>

const double LOOP_RATE_HZ = 30.0;

// RacingControlNode 类的构造函数
RacingControlNode::RacingControlNode(const std::string& node_name,const rclcpp::NodeOptions& options)
  : rclcpp::Node(node_name, options) {

  if (!msg_process_) {
    msg_process_ = std::make_shared<std::thread>(
        std::bind(&RacingControlNode::MessageProcess, this));
  }

  // 参数声明和获取 
  this->declare_parameter<std::string>("pub_control_topic", pub_control_topic_);
  this->get_parameter<std::string>("pub_control_topic", pub_control_topic_);

  // 巡线参数
  this->declare_parameter<float>("follow_linear_speed", follow_linear_speed_);
  this->get_parameter<float>("follow_linear_speed", follow_linear_speed_);
  this->declare_parameter<float>("follow_angular_ratio", follow_angular_ratio_);
  this->get_parameter<float>("follow_angular_ratio", follow_angular_ratio_);
  this->declare_parameter<float>("line_confidence_threshold", line_confidence_threshold_);
  this->get_parameter<float>("line_confidence_threshold", line_confidence_threshold_);

  // 避障参数
  this->declare_parameter<float>("avoid_linear_speed", avoid_linear_speed_);
  this->get_parameter<float>("avoid_linear_speed", avoid_linear_speed_);
  this->declare_parameter<float>("avoid_angular_ratio", avoid_angular_ratio_);
  this->get_parameter<float>("avoid_angular_ratio", avoid_angular_ratio_);
  this->declare_parameter<float>("obstacle_confidence_threshold", obstacle_confidence_threshold_);
  this->get_parameter<float>("obstacle_confidence_threshold", obstacle_confidence_threshold_);
  this->declare_parameter<float>("parking_sign_confidence_threshold", parking_sign_confidence_threshold_);
  this->get_parameter<float>("parking_sign_confidence_threshold", parking_sign_confidence_threshold_);
  this->declare_parameter<int>("bottom_threshold_caution", bottom_threshold_caution_);
  this->get_parameter<int>("bottom_threshold_caution", bottom_threshold_caution_);
  this->declare_parameter<int>("bottom_threshold_avoid", bottom_threshold_avoid_);
  this->get_parameter<int>("bottom_threshold_avoid", bottom_threshold_avoid_);
  this->declare_parameter<int>("parking_y_threshold", parking_y_threshold_);
  this->get_parameter<int>("parking_y_threshold", parking_y_threshold_);

  // 特殊状态下的参数
  this->declare_parameter<float>("cruise_linear_speed", cruise_linear_speed_);
  this->get_parameter<float>("cruise_linear_speed", cruise_linear_speed_);
  this->declare_parameter<float>("recovering_linear_speed", recovering_linear_speed_);
  this->get_parameter<float>("recovering_linear_speed", recovering_linear_speed_);
  this->declare_parameter<float>("recovering_angular_ratio", recovering_angular_ratio_);
  this->get_parameter<float>("recovering_angular_ratio", recovering_angular_ratio_);

  // 订阅者和发布者创建
  point_subscriber_ =
    this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      "racing_track_center_detection", 
      rclcpp::SensorDataQoS(),
      std::bind(&RacingControlNode::subscription_callback_point, this, std::placeholders::_1)); 

  target_subscriber_ =
    this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      "racing_obstacle_detection", 
      rclcpp::SensorDataQoS(),
      std::bind(&RacingControlNode::subscription_callback_target, this, std::placeholders::_1)); 

  // 订阅 /sign4return
  sign_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
    "/sign4return",
    rclcpp::SensorDataQoS(),
    std::bind(&RacingControlNode::sign_callback, this, std::placeholders::_1));
  
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(pub_control_topic_, 5);
  
  RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "RacingControlNode initialized!");
}

// 析构函数
RacingControlNode::~RacingControlNode(){
  if (msg_process_ && msg_process_->joinable()) {
    process_stop_ = true;
    msg_process_->join();
    msg_process_ = nullptr;
  }
}

// /sign4return 的回调函数
void RacingControlNode::sign_callback(const std_msgs::msg::Int32::SharedPtr msg) {
  int sign_value = msg->data;
  RCLCPP_INFO(this->get_logger(), "Received sign4return: %d", sign_value);
  
  if (sign_value == 5) {
    // 收到5时，挂起控制节点
    RCLCPP_WARN(this->get_logger(), "Suspending control node (manual override).");
    is_suspended_ = true;
    // 立即发布停止指令
    auto stop_msg = geometry_msgs::msg::Twist();
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    publisher_->publish(stop_msg);

  } else if (sign_value == 6) {
    // 收到6时，恢复控制
    RCLCPP_INFO(this->get_logger(), "Resuming control node.");
    is_suspended_ = false;
    // 将状态机重置为默认巡线状态，避免从异常状态恢复
    current_state_ = State::LINE_FOLLOWING;
    has_valid_twist_ = false; // 清除旧的指令历史
  }
}

// 中线消息回调
void RacingControlNode::subscription_callback_point(const ai_msgs::msg::PerceptionTargets::SharedPtr point_msg){
  RCLCPP_DEBUG(this->get_logger(), "Received a track center message.");
  {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    latest_point_msg_ = point_msg;
  }
}

// 障碍物消息回调
void RacingControlNode::subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg){
  RCLCPP_DEBUG(this->get_logger(), "Received an obstacle message.");
  {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    latest_targets_msg_ = targets_msg;
  }
}

// 核心控制逻辑线程
void RacingControlNode::MessageProcess(){
  rclcpp::Rate loop_rate(LOOP_RATE_HZ);

  while(process_stop_ == false){
    // 检查节点是否被挂起或已停车
    if (is_suspended_ || current_state_ == State::PARKED) {
      // 如果被挂起或已停车，则仅发布停止指令并等待
      if (current_state_ == State::PARKED) {
         auto stop_msg = geometry_msgs::msg::Twist(); // Ensure it stays stopped
         publisher_->publish(stop_msg);
      }
      loop_rate.sleep();
      continue;
    }

    std::unique_lock<std::mutex> lock(point_target_mutex_);
    auto current_line_msg = latest_point_msg_;
    auto current_obstacle_msg = latest_targets_msg_;
    lock.unlock();

    if (!current_line_msg) {
        RCLCPP_INFO(this->get_logger(), "Waiting for track center message...");
        loop_rate.sleep();
        continue;
    }

    // 最高优先级：检查是否满足停车条件
    if (isParkingConditionMet(current_obstacle_msg)) {
            RCLCPP_FATAL(this->get_logger(), "满足停车条件！正在停止。");
            current_state_ = State::PARKED;
            publisher_->publish(geometry_msgs::msg::Twist());
            continue;
    }
    
    // 第二优先级：检查是否有需要躲避的障碍物
    auto obstacle_to_avoid = findAvoidanceObstacle(current_obstacle_msg);
    // 检查是否有任何可以跟随的目标（线或停车标志）
    bool has_primary_target = hasValidTrackLine(current_line_msg) || findFollowableParkingSign(current_obstacle_msg).has_value();

    // --- 状态转换逻辑 ---
    bool should_force_avoidance = obstacle_to_avoid.has_value();
    if (current_state_ == State::RECOVERING_LINE && obstacle_to_avoid) {
        // 如果当前正在回线，只有当新障碍物出现在回线同侧时，才需要再次进入避障
        if (!isObstacleOnRecoverySide(*obstacle_to_avoid)) {
            RCLCPP_INFO(this->get_logger(), "回线时发现异侧障碍物，忽略，继续回线。");
            should_force_avoidance = false; // 覆盖决定，不强制避障
        }
    }

    if (should_force_avoidance && current_state_ != State::OBSTACLE_AVOIDING) {
        RCLCPP_INFO(this->get_logger(), "障碍物过近！强制切换到避障模式。");
        current_state_ = State::OBSTACLE_AVOIDING;
        has_valid_twist_ = false; // 旧指令失效
    }
    
    // 2. 根据当前状态执行动作
    switch(current_state_) {

      case State::OBSTACLE_AVOIDING:
        if (obstacle_to_avoid) {
          // 持续检测到障碍物，执行避障
          ObstaclesAvoiding(*obstacle_to_avoid);
        } else {
          // 结束避障，重置转弯方向
          last_avoidance_direction_ = 0.0f;
          RCLCPP_INFO(this->get_logger(), "Obstacle no longer detected. Deciding next state...");
          // 如果看得到主目标，就去巡线；否则就去寻找赛道
          if (has_primary_target) {
              current_state_ = State::LINE_FOLLOWING;
              RCLCPP_INFO(this->get_logger(), "Primary target found! Switching to LINE_FOLLOWING.");
          } else {
              current_state_ = State::RECOVERING_LINE;
              RCLCPP_INFO(this->get_logger(), "No clear target. Switching to RECOVERING_LINE.");
          }
        }
        break;

      case State::RECOVERING_LINE:
        last_avoidance_direction_ = 0.0f;
        RCLCPP_INFO(this->get_logger(), "In RECOVERING_LINE state, trying to find a target.");
        
        // 检查是否已找到任何主要目标（线或停车标志）
        if (has_primary_target) {
          // 找到了，切换回巡线状态
          current_state_ = State::LINE_FOLLOWING;
          RCLCPP_INFO(this->get_logger(), "Primary target (Line or Sign) found! Switching back to LINE_FOLLOWING.");
        } else {
          // 检查是否存在障碍物且 bottom >= bottom_threshold_caution_
          auto twist_msg = geometry_msgs::msg::Twist();
          bool slow_down_for_obstacle = false;
          // 我们需要检查是否有障碍物进入了谨慎区域
          if (current_obstacle_msg && !current_obstacle_msg->targets.empty()) {
              for (const auto& target : current_obstacle_msg->targets) {
                  if (target.type == "construction_cone" && !target.rois.empty()) {
                      int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
                      if (target.rois[0].confidence >= obstacle_confidence_threshold_ && bottom >= bottom_threshold_caution_) {
                          // 发现一个谨慎区障碍物，现在检查它是否在回线同侧
                          if (isObstacleOnRecoverySide(target)) {
                              slow_down_for_obstacle = true;
                              RCLCPP_WARN(this->get_logger(), "回线时，同侧发现谨慎区障碍物，减速！");
                              break; // 找到一个就够了
                          }
                      }
                  }
              }
          }
          twist_msg.linear.x = slow_down_for_obstacle ? avoid_linear_speed_ : recovering_linear_speed_;
          twist_msg.angular.z = -1.0 * std::copysign(recovering_angular_ratio_, last_avoidance_angular_z_);
          RCLCPP_INFO(this->get_logger(), "Recovering with linear speed: %.2f, angular z: %f", twist_msg.linear.x, twist_msg.angular.z);
          publisher_->publish(twist_msg);
        }
        break;

      case State::LINE_FOLLOWING:
        { 
          // 使用 isObstacleInCautionZone() 辅助函数决定速度
          float target_linear_speed = isObstacleInCautionZone(current_obstacle_msg) 
                                        ? avoid_linear_speed_ 
                                        : follow_linear_speed_;

          if (!LineFollowing(target_linear_speed)) {
            
            if (has_valid_twist_) {
                publisher_->publish(last_valid_twist_);
            } else {
                // 如果没有任何历史指令，则执行低速直行
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "LineFollowing failed. No valid target. Executing fallback.");
                auto cruise_msg = geometry_msgs::msg::Twist();
                cruise_msg.linear.x = cruise_linear_speed_;
                publisher_->publish(cruise_msg);
            }
          }
        }
        break;
      
      default: // 包括 STOP 状态
        publisher_->publish(geometry_msgs::msg::Twist()); // 发布停止指令
        break;
    }

    loop_rate.sleep();
    
  }
}

// 巡线控制函数
bool RacingControlNode::LineFollowing(float target_linear_speed){
  
  int target_x = 0;
  int target_y = 0;
  std::string log_reason;
  bool use_parking_sign = false;

  // 安全地获取最新消息
  std::unique_lock<std::mutex> lock(point_target_mutex_);
  auto current_line_msg = latest_point_msg_;
  auto current_obstacle_msg = latest_targets_msg_;
  lock.unlock();

  // 1. 决策使用哪个目标：优先使用停车标志
  auto followable_sign = findFollowableParkingSign(current_obstacle_msg);
  
  if (followable_sign) {
    // 如果找到了可以跟随的停车标志
    target_x = followable_sign->rois[0].rect.x_offset + followable_sign->rois[0].rect.width / 2;
    target_y = followable_sign->rois[0].rect.y_offset + followable_sign->rois[0].rect.height / 2;
    use_parking_sign = true;
    log_reason = "Parking Sign";
  } else if (hasValidTrackLine(current_line_msg)) {
    // 否则，如果找到了有效的赛道线
    target_x = static_cast<int>(current_line_msg->targets[0].points[0].point[0].x);
    target_y = static_cast<int>(current_line_msg->targets[0].points[0].point[0].y);
    log_reason = "Track Line";
  } else {
    // 如果两者都找不到，则巡线失败
    return false;
  }
  
  // 2. 如果找到了目标，则计算并发布控制指令
  float center_offset = static_cast<float>(target_x) - 320.0f;
  float target_y_relative = (static_cast<float>(target_y) - 256.0f) / (480.0f - 256.0f);
  
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = target_linear_speed;

  twist_msg.angular.z = (use_parking_sign ? 2.0f : 1.0f) * follow_angular_ratio_ * (center_offset / 320.0f) * target_y_relative;
  
  last_valid_twist_ = twist_msg;
  last_valid_twist_.linear.x = target_linear_speed; 
  last_valid_twist_.angular.z = twist_msg.angular.z;
  has_valid_twist_ = true;

  publisher_->publish(twist_msg);
  
  RCLCPP_INFO(this->get_logger(), "Following %s -> X:%d, Y:%d, Ang:%.2f, Lin:%.2f", 
              log_reason.c_str(), target_x, target_y, twist_msg.angular.z, target_linear_speed);

  return true;
}

// 避障控制函数
void RacingControlNode::ObstaclesAvoiding(const ai_msgs::msg::Target &target){
  if (target.rois.empty() || target.rois[0].rect.width == 0) {
      RCLCPP_ERROR(this->get_logger(), "CRITICAL: ObstaclesAvoiding called with invalid ROI data!");
      return; 
  }

  auto twist_msg = geometry_msgs::msg::Twist();
  int center_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
  float obstacle_center_offset = static_cast<float>(center_x) - 320.0f; 

  float angular_z_avoid = 0.0f;

  if(obstacle_center_offset < 5.0f && obstacle_center_offset >= 0) {
    obstacle_center_offset = 5.0f;
  } else if(obstacle_center_offset > -5.0f && obstacle_center_offset < 0) {
    obstacle_center_offset = -5.0f;
  }

  angular_z_avoid = -1.0f * avoid_angular_ratio_ * std::min(4.0f, (320.0f / obstacle_center_offset));

  // 记录这次避障的转向角速度，以备恢复赛道时使用
  last_avoidance_angular_z_ = angular_z_avoid;

  //记录这次避障的方向
  if(last_avoidance_direction_ == 0.0) //每次结束避障时更新last_avoidance_direction_ = 0 ，仅在第一次进行记录
  {
    if (angular_z_avoid > 0)
    {
      last_avoidance_direction_ = 1.0f;
    }
    else
    {
      last_avoidance_direction_ = -1.0f;
    }
  }

  if( last_avoidance_direction_ * angular_z_avoid < 0 && last_avoidance_direction_ != 0) //避障状态机中出现于第一次记录方向相反，则用上一次状态
  {
    angular_z_avoid = 10.0f * last_avoidance_direction_;
  }


  twist_msg.linear.x = avoid_linear_speed_;
  twist_msg.angular.z = angular_z_avoid;
  publisher_->publish(twist_msg);
  RCLCPP_INFO(this->get_logger(), "Obstacles Avoiding -> CenterX:%d, Ang_Z: %f, Lin_X: %f", center_x, angular_z_avoid, avoid_linear_speed_);
}

// 检查是否有有效的、高置信度的赛道线
bool RacingControlNode::hasValidTrackLine(const ai_msgs::msg::PerceptionTargets::SharedPtr& line_msg) const {
    return line_msg && !line_msg->targets.empty() &&
           !line_msg->targets[0].points.empty() &&
           !line_msg->targets[0].points[0].confidence.empty() &&
           line_msg->targets[0].points[0].confidence[0] >= line_confidence_threshold_;
}

// 查找一个可以用来跟随的停车标志
std::optional<ai_msgs::msg::Target> RacingControlNode::findFollowableParkingSign(const ai_msgs::msg::PerceptionTargets::SharedPtr& targets_msg) const {
    if (targets_msg && !targets_msg->targets.empty()) {
        for (const auto& target : targets_msg->targets) {
            if (target.type == "parking_sign" && !target.rois.empty() &&
                target.rois[0].confidence >= parking_sign_confidence_threshold_) {
                return target; // 找到了，返回这个目标
            }
        }
    }
    return std::nullopt; // 没找到
}

// 检查是否满足最终停车条件
bool RacingControlNode::isParkingConditionMet(const ai_msgs::msg::PerceptionTargets::SharedPtr& targets_msg) const {
    if (auto sign = findFollowableParkingSign(targets_msg)) {
        // 首先要有一个能跟随的标志，然后检查它是否足够近
        int sign_center_y = sign->rois[0].rect.y_offset + sign->rois[0].rect.height / 2;
        return sign_center_y >= parking_y_threshold_;
    }
    return false;
}

// 查找需要紧急避障的障碍物
std::optional<ai_msgs::msg::Target> RacingControlNode::findAvoidanceObstacle(const ai_msgs::msg::PerceptionTargets::SharedPtr& targets_msg) const {
    if (targets_msg && !targets_msg->targets.empty()) {
        for (const auto& target : targets_msg->targets) {
            if (target.type == "construction_cone" && !target.rois.empty()) {
                int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
                // 判断条件：置信度足够高 且 底部超过了避障阈值
                if (target.rois[0].confidence >= obstacle_confidence_threshold_ && bottom >= bottom_threshold_avoid_) {
                    return target; // 找到了，返回它
                }
            }
        }
    }
    return std::nullopt;
}

// 检查是否有障碍物进入了“谨慎区域”（需要减速）
bool RacingControlNode::isObstacleInCautionZone(const ai_msgs::msg::PerceptionTargets::SharedPtr& targets_msg) const {
    if (targets_msg && !targets_msg->targets.empty()) {
        for (const auto& target : targets_msg->targets) {
            if (target.type == "construction_cone" && !target.rois.empty()) {
                int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
                // 判断条件：置信度足够高 且 底部超过了谨慎阈值
                if (target.rois[0].confidence >= obstacle_confidence_threshold_ && bottom >= bottom_threshold_caution_) {
                    return true; // 找到了
                }
            }
        }
    }
    return false;
}

// 检查障碍物是否在回线方向的同侧
bool RacingControlNode::isObstacleOnRecoverySide(const ai_msgs::msg::Target& obstacle) const {
    if (last_avoidance_angular_z_ == 0.0f || obstacle.rois.empty()) {
        // 如果没有记录避障方向，或者障碍物信息无效，默认认为不在同侧（安全起见，不触发特殊逻辑）
        return false;
    }

    int obstacle_center_x = obstacle.rois[0].rect.x_offset + obstacle.rois[0].rect.width / 2;
    const int screen_center_x = 320;

    // last_avoidance_angular_z_ < 0 表示上次向左转避障，现在需要向右回线
    // 此时我们只关心右侧 (x > 320) 的新障碍物
    bool recovering_right = last_avoidance_angular_z_ > 0;
    if (recovering_right && obstacle_center_x > screen_center_x) {
        return true;
    }

    // last_avoidance_angular_z_ > 0 表示上次向右转避障，现在需要向左回线
    // 此时我们只关心左侧 (x < 320) 的新障碍物
    bool recovering_left = last_avoidance_angular_z_ < 0;
    if (recovering_left && obstacle_center_x < screen_center_x) {
        return true;
    }

    // 其他情况（如向右回线时，障碍物在左边）都返回 false
    return false;
}

// 主函数
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacingControlNode>("racing_control"));
  rclcpp::shutdown();
  RCLCPP_WARN(rclcpp::get_logger("racing_control"), "Pkg exit.");
  return 0;
}
