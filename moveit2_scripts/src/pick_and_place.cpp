#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class RobotArm {
public:
    RobotArm() {
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

        executor.add_node(move_group_node);
        std::thread([this]() { this->executor.spin(); }).detach();

        move_group_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP_ARM);
        joint_model_group_arm = move_group_arm->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
        
        move_group_gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP_GRIPPER);
        joint_model_group_gripper = move_group_gripper->getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);

        RCLCPP_INFO(LOGGER, "---------------------------INSTANTIATED!");
    }

    void move_joint_space(float angle_0, float angle_1, float angle_2, float angle_3, float angle_4, float angle_5) {
        moveit::core::RobotStatePtr current_state = move_group_arm->getCurrentState(10);

        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group_arm, joint_group_positions);

        joint_group_positions[0] = angle_0; // Shoulder Pan
        joint_group_positions[1] = angle_1; // Shoulder Lift
        joint_group_positions[2] = angle_2; // Elbow
        joint_group_positions[3] = angle_3; // Wrist 1
        joint_group_positions[4] = angle_4; // Wrist 2
        joint_group_positions[5] = angle_5; // Wrist 3

        move_group_arm->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_arm->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_arm->execute(my_plan);
        } else {
            RCLCPP_ERROR(LOGGER, "Joint space movement plan failed!");
        }
    }

    void move_end_effector(double x, double y, double z, double roll_deg, double pitch_deg, double yaw_deg) {
        geometry_msgs::msg::Pose target_pose;
        tf2::Quaternion q;
        q.setRPY(roll_deg * M_PI/180.0, pitch_deg * M_PI/180.0, yaw_deg * M_PI/180.0);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        move_group_arm->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_arm->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_arm->execute(my_plan);
        } else {
            RCLCPP_ERROR(LOGGER, "End effector movement plan failed!");
        }
    }

    void move_waypoint(double delta, const std::string& direction) {
        RCLCPP_INFO(LOGGER, "Move waypoint!");

        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose target_pose = move_group_arm->getCurrentPose().pose;

        if(direction == "x") {
            target_pose.position.x += delta;
        } else if(direction == "y") {
            target_pose.position.y += delta;
        } else if(direction == "z") {
            target_pose.position.z += delta;
        } else {
            RCLCPP_ERROR(LOGGER, "Invalid direction!");
            return;
        }

        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;

        double fraction = move_group_arm->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if(fraction >= 0.0) {
            move_group_arm->execute(trajectory);
        } else {
            RCLCPP_ERROR(LOGGER, "Waypoint movement plan failed!");
        }
    }

    void cmd_arm(const std::string& target_name) {
        RCLCPP_INFO(LOGGER, target_name.c_str());

        move_group_arm->setNamedTarget(target_name);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
        bool success_arm = (move_group_arm->plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);

        if(success_arm) {
            move_group_arm->execute(my_plan_arm);
        } else {
            RCLCPP_ERROR(LOGGER, "Arm plan failed!");
        }
    }

    void cmd_gripper(const std::string& target_name) {
        RCLCPP_INFO(LOGGER, target_name.c_str());

        move_group_gripper->setNamedTarget(target_name);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
        bool success_gripper = (move_group_gripper->plan(my_plan_gripper) == moveit::core::MoveItErrorCode::SUCCESS);

        if(success_gripper) {
            move_group_gripper->execute(my_plan_gripper);
        } else {
            RCLCPP_ERROR(LOGGER, "Gripper plan failed!");
        }
    }

private:
    rclcpp::Node::SharedPtr move_group_node;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm;
    const moveit::core::JointModelGroup* joint_model_group_arm;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper;
    const moveit::core::JointModelGroup* joint_model_group_gripper;
    rclcpp::executors::SingleThreadedExecutor executor;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    RobotArm robotArm;

    robotArm.cmd_arm("home");
    robotArm.cmd_gripper("gripper_open");
    // Shoulder Pan | Shoulder Lift | Elbow | Wrist 1 | Wrist 2 | Wrist 3
    robotArm.move_joint_space(3.47, -1.64, -1.59, -1.47, 1.56, -5.70);
    robotArm.cmd_gripper("gripper_close");
    // robotArm.move_end_effector(0.2, -0.34, 0.28, 180.0, 0.0, 0.0);
    robotArm.move_waypoint(-0.1, "z");
    robotArm.move_waypoint(0.1, "x");
    robotArm.move_waypoint(0.2, "y");

    rclcpp::shutdown();
    return 0;
}