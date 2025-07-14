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
  this->declare_parameter<int>("bottom_threshold", bottom_threshold_);
  this->get_parameter<int>("bottom_threshold", bottom_threshold_);

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
    // 检查节点是否被挂起
    if (is_suspended_) {
      // 如果被挂起，则不执行任何控制逻辑，仅循环等待
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

    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;

    // 状态机决策
    bool obstacle_detected_and_close = false;
    ai_msgs::msg::Target relevant_obstacle_target;

    // 1. 检查障碍物，这具有最高优先级
    if (current_obstacle_msg && !current_obstacle_msg->targets.empty()) {
        for(const auto &target : current_obstacle_msg->targets){
            if(target.type == "construction_cone" && !target.rois.empty()){
                float obstacle_conf = target.rois[0].confidence;
                int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
                if (obstacle_conf >= obstacle_confidence_threshold_ && bottom >= bottom_threshold_) {
                    obstacle_detected_and_close = true;
                    relevant_obstacle_target = target;
                    // 如果当前不是避障状态，则切换进去
                    if (current_state_ != State::OBSTACLE_AVOIDING) {
                        RCLCPP_INFO(this->get_logger(), "Close obstacle detected! Switching to OBSTACLE_AVOIDING.");
                        current_state_ = State::OBSTACLE_AVOIDING;
                    }
                    break;
                }
            }
        }
    }
    
    // 2. 根据当前状态执行动作
    switch(current_state_) {

      case State::OBSTACLE_AVOIDING:
        if (obstacle_detected_and_close) {
          // 持续检测到障碍物，执行避障
          ObstaclesAvoiding(relevant_obstacle_target);
        } else {
          // 结束避障，重置转弯方向
          last_avoidance_direction_ = 0.0f;
          RCLCPP_INFO(this->get_logger(), "Obstacle no longer detected. Deciding next state...");

          // 优化点 1: 检查是否存在任何主要目标（线或停车标志）
          if (hasVisiblePrimaryTarget()) {
              // 看到了清晰的赛道线或停车标志，切回巡线模式
              current_state_ = State::LINE_FOLLOWING;
              RCLCPP_INFO(this->get_logger(), "Primary target (Line or Sign) found! Switching to LINE_FOLLOWING.");
          } else {
              // 两个都没看到，进入寻找赛道状态
              current_state_ = State::RECOVERING_LINE;
              RCLCPP_INFO(this->get_logger(), "No clear target. Switching to RECOVERING_LINE.");
          }
        }
        break;

      case State::RECOVERING_LINE:
        RCLCPP_INFO(this->get_logger(), "In RECOVERING_LINE state, trying to find a target.");
        
        // 优化点 2: 检查是否已找到任何主要目标（线或停车标志）
        if (hasVisiblePrimaryTarget()) {
          // 找到了，切换回巡线状态
          current_state_ = State::LINE_FOLLOWING;
          RCLCPP_INFO(this->get_logger(), "Primary target (Line or Sign) found! Switching back to LINE_FOLLOWING.");
        } else {
          // 未找到，执行“反向转弯”寻找赛道
          twist_msg.linear.x = recovering_linear_speed_;
          twist_msg.angular.z = -1.0 * std::copysign(recovering_angular_ratio_, last_avoidance_angular_z_);
          RCLCPP_INFO(this->get_logger(), "Recovering with Ang_Z: %f", twist_msg.angular.z);
          publisher_->publish(twist_msg);
        }
        break;

      case State::LINE_FOLLOWING:
        // 在巡线状态，也要检查障碍物，因为障碍物可能突然出现
        if (obstacle_detected_and_close) {
            current_state_ = State::OBSTACLE_AVOIDING;
            has_valid_twist_ = false; // 进入避障时，旧的巡线指令失效
            ObstaclesAvoiding(relevant_obstacle_target);
        } 
        else {
          // 正常巡线逻辑 (这部分逻辑保持不变, 它已经很健壮了)
          float line_confidence = 0.0f;
          if (!current_line_msg->targets.empty() && 
              !current_line_msg->targets[0].points.empty() && 
              !current_line_msg->targets[0].points[0].confidence.empty()) {
            line_confidence = current_line_msg->targets[0].points[0].confidence[0];
          }

          const auto& line_target_to_pass = current_line_msg->targets.empty() ? ai_msgs::msg::Target() : current_line_msg->targets[0];
          bool target_followed = LineFollowing(line_target_to_pass, line_confidence);

          if (!target_followed) {
            if (has_valid_twist_) {
                RCLCPP_WARN(this->get_logger(), "No valid target found. Reusing last valid command.");
                publisher_->publish(last_valid_twist_);
            } else {
                RCLCPP_WARN(this->get_logger(), "No valid target and no history. Cruising straight.");
                twist_msg.linear.x = cruise_linear_speed_;
                twist_msg.angular.z = 0.0;
                publisher_->publish(twist_msg);
            }
          }
        }
        break;
      
      default: // 包括 STOP 状态
        publisher_->publish(twist_msg); // 发布停止指令
        break;
    }

    loop_rate.sleep();
    
  }
}

// 巡线控制函数
bool RacingControlNode::LineFollowing(const ai_msgs::msg::Target &line_target, float line_confidence){
  
  bool use_parking_sign = false;
  int target_x = 0;
  int target_y = 0;
  std::string log_reason;
  bool target_found = false;

  // 1. 优先检查是否存在高置信度的 "parking_sign"
  {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    if (latest_targets_msg_ && !latest_targets_msg_->targets.empty()) {
      for (const auto& target : latest_targets_msg_->targets) {
        // 条件判断：类型是 parking_sign，有 ROI，且置信度大于等于阈值
        if (target.type == "parking_sign" && !target.rois.empty() && 
            target.rois[0].confidence >= parking_sign_confidence_threshold_) {
          
          target_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
          target_y = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
          use_parking_sign = true;
          log_reason = "High-Conf Parking Sign"; // 更新日志信息，更明确
          break; 
        }
      }
    }
  }

  // 2. 决策最终使用哪个目标
  if (use_parking_sign) {
    // 如果找到了高置信度的停车标志，我们认为这是一个有效的目标
    target_found = true;
  } else if (line_confidence >= line_confidence_threshold_) {
    // 如果没有停车标志，但赛道线置信度高，且数据有效
    if (!line_target.points.empty() && !line_target.points[0].point.empty()) {
        target_x = static_cast<int>(line_target.points[0].point[0].x);
        target_y = static_cast<int>(line_target.points[0].point[0].y);
        log_reason = "High-Conf Line";
        target_found = true;
    }
  }

  // 3. 如果最终没有找到任何有效目标，则返回 false
  if (!target_found) {
    // 这将告诉 MessageProcess 执行备用方案
    return false;
  }
  
  // 4. 如果找到了目标，则计算并发布控制指令
  float center_offset = static_cast<float>(target_x) - 320.0f;
  if (std::abs(center_offset) < 5.0f) { center_offset = 0.0f; }
  
  auto twist_msg = geometry_msgs::msg::Twist();
  float target_y_relative = (static_cast<float>(target_y) - 256.0f) / (480.0f - 256.0f);
  target_y_relative = std::max(0.0f, std::min(1.0f, target_y_relative));
  
  float angular_z = 0.0f;
  if (use_parking_sign) {
    angular_z = 3.0f * follow_angular_ratio_ * (center_offset / 320.0f) * target_y_relative; 
  } else {
    angular_z = follow_angular_ratio_ * (center_offset / 320.0f) * target_y_relative; 
  }

  twist_msg.linear.x = follow_linear_speed_;
  twist_msg.angular.z = angular_z;

  // 存储这个有效的指令
  last_valid_twist_ = twist_msg;
  last_valid_twist_.linear.x = cruise_linear_speed_; 
  last_valid_twist_.angular.z = std::copysign(10.0f, angular_z);
  has_valid_twist_ = true;

  publisher_->publish(twist_msg);
  float sign_conf = use_parking_sign ? latest_targets_msg_->targets[0].rois[0].confidence : 0.0f;
  RCLCPP_INFO(this->get_logger(), "Following %s -> X:%d, Y:%d, Ang_Z: %f, Lin_X: %f (LineConf: %.2f, SignConf: %.2f)", 
              log_reason.c_str(), target_x, target_y, angular_z, follow_linear_speed_, line_confidence, sign_conf);

  // 成功发布指令，返回 true
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

bool RacingControlNode::hasVisiblePrimaryTarget() {
    std::unique_lock<std::mutex> lock(point_target_mutex_);

    // 1. Check for a high-confidence track line
    if (latest_point_msg_ && !latest_point_msg_->targets.empty() && 
        !latest_point_msg_->targets[0].points.empty() && 
        !latest_point_msg_->targets[0].points[0].confidence.empty() &&
        latest_point_msg_->targets[0].points[0].confidence[0] >= line_confidence_threshold_) {
        return true; // Found a valid line
    }

    // 2. If no line, check for a high-confidence parking sign
    if (latest_targets_msg_ && !latest_targets_msg_->targets.empty()) {
        for (const auto& target : latest_targets_msg_->targets) {
            if (target.type == "parking_sign" && !target.rois.empty() &&
                target.rois[0].confidence >= parking_sign_confidence_threshold_) {
                return true; // Found a valid parking sign
            }
        }
    }

    // 3. If neither is found
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
