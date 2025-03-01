#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <iostream>
#include <Eigen/Dense>

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_urdf>" << std::endl;
        return -1;
    }

    std::string urdf_path = argv[1];

    // Load the model from URDF
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::Data data(model);

    // Set q to zero
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

    // Perform forward kinematics with q = 0
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    // 输出所有 joint 位置
    std::cout << "Joint positions with q set to zero:" << std::endl;
    for (size_t i = 0; i < model.njoints; ++i) {
        std::cout << "Joint " << model.names[i] << ": " 
                  << data.oMi[i].translation().transpose() << std::endl;
    }

    // 查询 contact_L_Link 和 contact_R_Link 的位置
    pinocchio::FrameIndex contactLId = model.getFrameId("foot_L_Joint");
    pinocchio::FrameIndex contactRId = model.getFrameId("contact_R_Link");
    if (contactLId >= model.nframes || contactRId >= model.nframes) {
        std::cerr << "Cannot find one or both contact frames in the model." << std::endl;
    } else {
        std::cout << "foot_L_Joint position: " 
                  << data.oMf[contactLId].translation().transpose() << std::endl;
        std::cout << "contact_R_Link position: " 
                  << data.oMf[contactRId].translation().transpose() << std::endl;
    }

    // 使用随机 q 进行前向动力学计算
    q = pinocchio::randomConfiguration(model);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    std::cout << "\nJoint positions with q set to random values:" << std::endl;
    for (size_t i = 0; i < model.njoints; ++i) {
        std::cout << "Joint " << model.names[i] << ": " 
                  << data.oMi[i].translation().transpose() << std::endl;
    }
    // 查询随机 q 下的 contact 链位置
    if (contactLId < model.nframes && contactRId < model.nframes) {
        std::cout << "foot_L_Joint position (random q): " 
                  << data.oMf[contactLId].translation().transpose() << std::endl;
        std::cout << "contact_R_Link position (random q): " 
                  << data.oMf[contactRId].translation().transpose() << std::endl;
    }

    return 0;
}