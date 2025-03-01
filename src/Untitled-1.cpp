#include <Eigen/Dense>
#include <vector>
#include <iostream>

int main() {
    // 初始化一个 std::vector<float>
    std::vector<float> q = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};

    // 将 std::vector<float> 转换为 Eigen::VectorXd
    Eigen::VectorXd jointPositions = Eigen::Map<Eigen::VectorXd>(q.data(), q.size());

    // 输出转换后的 Eigen::VectorXd
    std::cout << "jointPositions: " << jointPositions.transpose() << std::endl;

    return 0;
}