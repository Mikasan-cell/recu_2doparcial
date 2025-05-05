#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>
#include <vector>

class PandaArmController {
public:
    PandaArmController(rclcpp::Node::SharedPtr node)
        : node_(node),
          move_group_interface_(node, "panda_arm") {}

    void run() {
        int choice;
        while (rclcpp::ok()) {
            print_menu();
            std::cin >> choice;
            std::vector<double> joint_positions;

            switch (choice) {
                case 1:
                    joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 3.14, 0.78};
                    move_to_joint_positions(joint_positions);
                    break;
                case 2:
                    joint_positions = {0.0, 1.57, 0.0, 0.0, 0.0, 3.14, 0.78};
                    move_to_joint_positions(joint_positions);
                    break;
                case 3:
                    joint_positions = get_user_joint_positions();
                    if (!joint_positions.empty()) {
                        move_to_joint_positions(joint_positions);
                    }
                    break;
                case 4:
                    RCLCPP_INFO(node_->get_logger(), "Hasta luego 👋");
                    return;
                default:
                    RCLCPP_WARN(node_->get_logger(), "Opción no válida. Intenta otra vez.");
                    break;
            }
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_interface_;

    void print_menu() {
        std::cout << "\n==============================" << std::endl;
        std::cout << "  🚀 MENÚ DE CONTROL DEL BRAZO" << std::endl;
        std::cout << "==============================" << std::endl;
        std::cout << "  [1] Mover a posición predefinida 1" << std::endl;
        std::cout << "  [2] Mover a posición predefinida 2" << std::endl;
        std::cout << "  [3] Ingresar posiciones personalizadas" << std::endl;
        std::cout << "  [4] Salir\n" << std::endl;
        std::cout << "  Selecciona una opción >>> ";
    }

    std::vector<double> get_user_joint_positions() {
        std::vector<double> positions(7);
        std::cout << "\nIntroduce 7 posiciones articulares (radianes):" << std::endl;
        for (int i = 0; i < 7; ++i) {
            std::cout << "  Articulación " << i + 1 << ": ";
            std::cin >> positions[i];
        }
        return positions;
    }

    void move_to_joint_positions(const std::vector<double>& positions) {
        if (positions.size() != 7) {
            RCLCPP_ERROR(node_->get_logger(), "Se requieren exactamente 7 valores.");
            return;
        }

        move_group_interface_.setJointValueTarget(positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (move_group_interface_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            move_group_interface_.execute(plan);
            RCLCPP_INFO(node_->get_logger(), "✅ Movimiento ejecutado con éxito.");
            RCLCPP_INFO(node_->get_logger(), "📌 Posición objetivo de las articulaciones:");
            for (size_t i = 0; i < positions.size(); ++i) {
                RCLCPP_INFO(node_->get_logger(), "  Articulación %zu: %.3f", i + 1, positions[i]);
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "❌ Falló la planificación.");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("panda_arm_controller");
    PandaArmController controller(node);
    controller.run();
    rclcpp::shutdown();
    return 0;
}

