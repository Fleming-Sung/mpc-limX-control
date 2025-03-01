#include "pinocchio_kinematics.h"

int main(int argc, char **argv) {

    PinocchioKinematics kinematics;

    // Set q to zero
    Eigen::VectorXd q = Eigen::VectorXd::Zero(kinematics.model.nq);
    kinematics.printJointPositions(q);

    // 查询 contact_L_Link 和 contact_R_Link 的位置
    Eigen::Vector3d contactLPos = kinematics.getLinkPosition("contact_L_Link");
    Eigen::Vector3d contactRPos = kinematics.getLinkPosition("contact_R_Link");
    std::cout << "contact_L_Link position: " << contactLPos.transpose() << std::endl;
    std::cout << "contact_R_Link position: " << contactRPos.transpose() << std::endl;

    // 使用随机 q 进行前向动力学计算
    q = pinocchio::randomConfiguration(kinematics.model);
    kinematics.printJointPositions(q);

    // 查询随机 q 下的 contact 链位置
    contactLPos = kinematics.getLinkPosition("contact_L_Link");
    contactRPos = kinematics.getLinkPosition("contact_R_Link");
    std::cout << "contact_L_Link position (random q): " << contactLPos.transpose() << std::endl;
    std::cout << "contact_R_Link position (random q): " << contactRPos.transpose() << std::endl;

    // 逆运动学计算
    Eigen::Vector3d target_position(0.1, 0.1, 0.1);
    Eigen::VectorXd initial_guess = Eigen::VectorXd::Zero(kinematics.model.nq);
    Eigen::VectorXd ik_solution = kinematics.inverseKinematics("contact_L_Link", target_position, initial_guess);
    std::cout << "Inverse kinematics solution: " << ik_solution.transpose() << std::endl;

    // 设置 base_link 的位姿
    Eigen::Vector3d base_position(0.0, 0.0, 0.5);
    Eigen::Quaterniond base_orientation(1.0, 0.0, 0.0, 0.0); // Identity quaternion
    kinematics.setBaseLinkPose(base_position, base_orientation);

    // 再次查询 contact_L_Link 和 contact_R_Link 的位置
    contactLPos = kinematics.getLinkPosition("contact_L_Link");
    contactRPos = kinematics.getLinkPosition("contact_R_Link");
    std::cout << "contact_L_Link position after setting base_link pose: " << contactLPos.transpose() << std::endl;
    std::cout << "contact_R_Link position after setting base_link pose: " << contactRPos.transpose() << std::endl;

    return 0;
}