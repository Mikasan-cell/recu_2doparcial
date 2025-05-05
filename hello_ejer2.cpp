#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>

class PandaArmController {
public:
    PandaArmController(rclcpp::Node::SharedPtr node) 
        : node_(node),
          move_group_interface_(node, "panda_arm"),
          planning_scene_interface_() {
        move_group_interface_.setPlanningTime(15.0);
        move_group_interface_.setNumPlanningAttempts(20);
        move_group_interface_.setMaxVelocityScalingFactor(0.3);
        move_group_interface_.setMaxAccelerationScalingFactor(0.3);
        move_group_interface_.setPlannerId("RRTConnectkConfigDefault");
        move_group_interface_.setGoalPositionTolerance(0.01);
        move_group_interface_.setGoalOrientationTolerance(0.1);

        RCLCPP_INFO(node_->get_logger(), "✅ Controlador Panda inicializado. Frame: %s", 
                   move_group_interface_.getPlanningFrame().c_str());

        add_obstacles();
    }

    void run() {
        int choice;
        while (rclcpp::ok()) {
            print_menu();
            if (!(std::cin >> choice)) {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                RCLCPP_WARN(node_->get_logger(), "⚠️ Entrada no válida.");
                continue;
            }

            switch (choice) {
                case 1:
                    move_to_pose(-0.58, 0.2, 0.5, 0.5);
                    break;
                case 2:
                    move_to_pose(0.7, 0.4, 0.2, 0.0);
                    break;
                case 3:
                    move_to_pose(0.1, 0.2, 0.0, 0.0);
                    break;
                case 4:
                    get_user_pose_and_move();
                    break;
                case 5:
                    RCLCPP_INFO(node_->get_logger(), "👋 Hasta luego.");
                    return;
                default:
                    RCLCPP_WARN(node_->get_logger(), "⚠️ Opción no válida. Intenta otra vez.");
                    break;
            }
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    void print_menu() {
        std::cout << "\n=========================================" << std::endl;
        std::cout << "     🤖 MENÚ DE CONTROL DEL BRAZO PANDA" << std::endl;
        std::cout << "=========================================" << std::endl;
        std::cout << "  [1] Mover a posición predefinida 1" << std::endl;
        std::cout << "  [2] Mover a posición predefinida 2" << std::endl;
        std::cout << "  [3] Mover a posición predefinida 3" << std::endl;
        std::cout << "  [4] Ingresar posición personalizada" << std::endl;
        std::cout << "  [5] Salir\n" << std::endl;
        std::cout << "  👉 Selecciona una opción >>> ";
    }

    void get_user_pose_and_move() {
        double x, y, z, w;
        std::cout << "\nIntroduce la posición deseada en coordenadas cartesianas:" << std::endl;
        std::cout << "  x: "; std::cin >> x;
        std::cout << "  y: "; std::cin >> y;
        std::cout << "  z: "; std::cin >> z;
        std::cout << "  w (orientación cuaternión): "; std::cin >> w;

        move_to_pose(x, y, z, w);
    }

    void move_to_pose(double x, double y, double z, double w) {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        q.setW(w);
        q.normalize();

        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();

        RCLCPP_INFO(node_->get_logger(), "🎯 Intentando mover a: x=%.2f, y=%.2f, z=%.2f", x, y, z);

        move_group_interface_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        auto start_time = node_->now();
        auto result = move_group_interface_.plan(plan);
        auto planning_time = (node_->now() - start_time).seconds();

        RCLCPP_INFO(node_->get_logger(), "⏱️ Tiempo de planificación: %.2f segundos", planning_time);

        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "✅ Planificación exitosa. Ejecutando movimiento...");
            move_group_interface_.execute(plan);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "❌ No se pudo alcanzar la pose especificada. Volviendo al menú...");
        }
    }

    void add_obstacles() {
        moveit_msgs::msg::CollisionObject obstacle1, obstacle2;

        obstacle1.id = "obstacle1";
        obstacle1.header.frame_id = move_group_interface_.getPlanningFrame();

        shape_msgs::msg::SolidPrimitive primitive1;
        primitive1.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive1.dimensions = {0.1, 0.1, 0.9};  // MÁS ALTA

        geometry_msgs::msg::Pose pose1;
        pose1.position.x = 0.3;
        pose1.position.y = 0.2;
        pose1.position.z = 0.45;  // Centro de la altura

        obstacle1.primitives.push_back(primitive1);
        obstacle1.primitive_poses.push_back(pose1);
        obstacle1.operation = moveit_msgs::msg::CollisionObject::ADD;

        obstacle2.id = "obstacle2";
        obstacle2.header.frame_id = move_group_interface_.getPlanningFrame();

        shape_msgs::msg::SolidPrimitive primitive2;
        primitive2.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive2.dimensions = {0.1, 0.1, 0.9};  // MÁS ALTA

        geometry_msgs::msg::Pose pose2;
        pose2.position.x = -0.4;
        pose2.position.y = -0.25;
        pose2.position.z = 0.45;

        obstacle2.primitives.push_back(primitive2);
        obstacle2.primitive_poses.push_back(pose2);
        obstacle2.operation = moveit_msgs::msg::CollisionObject::ADD;

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(obstacle1);
        collision_objects.push_back(obstacle2);

        planning_scene_interface_.applyCollisionObjects(collision_objects);
        RCLCPP_INFO(node_->get_logger(), "🧱 Obstáculos añadidos a la escena");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("panda_arm_controller");

    rclcpp::sleep_for(std::chrono::seconds(2));

    PandaArmController controller(node);
    controller.run();

    rclcpp::shutdown();
    return 0;
}

