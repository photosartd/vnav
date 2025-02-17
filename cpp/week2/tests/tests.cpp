#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <tuple>

TEST(TestRotation, TestMatrixValidation)
{
    Eigen::MatrixXd m(3, 3);
    m << 0.36, -0.48, 0.80,
        0.80, 0.60, -0.00,
        -0.48, -0.64, 0.60;
    EXPECT_TRUE((m * m.transpose()).isApprox(Eigen::MatrixXd::Identity(3, 3), 1e-2))
        << "m * m.transpose() did not approximate the identity matrix within the tolerance of 1e-5";
    auto determinant = m.determinant();
    EXPECT_NEAR(determinant, 1.0, 1e-2) << "m.determinant() did not return 1 within the tolerance of 1e-2";
}

Eigen::MatrixXd getRotationMatrixFromAngle(double angle)
{
    Eigen::MatrixXd m(2, 2);
    m << std::cos(angle), -std::sin(angle),
        std::sin(angle), std::cos(angle);
    return m;
}

TEST(TestRotation, Test2DRotationValid)
{
    double angleAlpha = M_PI / 6;
    double angleBeta = std::fmod(-M_PI / 2, 2 * M_PI);
    double anglePhi = std::fmod(angleAlpha + angleBeta, 2 * M_PI);
    auto rAlpha = getRotationMatrixFromAngle(angleAlpha);
    auto rBeta = getRotationMatrixFromAngle(angleBeta);
    auto rPhi = getRotationMatrixFromAngle(anglePhi);

    EXPECT_TRUE((rAlpha * rBeta).isApprox(rPhi, 1e-2)) << "rAplha * rBeta != rPhi";
}

TEST(TestRotation, TestRandomToRotation)
{
    Eigen::Matrix3d m = Eigen::Matrix3d::Random();

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R = U * V.transpose();

    if (R.determinant() < 0.0)
    {
        V.col(2) *= -1;
        R = U * V.transpose();
    }
    EXPECT_TRUE((R * R.transpose()).isApprox(Eigen::Matrix3d::Identity(3, 3), 1e-2)) << "R was not a rotation matrix after SVD";
}

Eigen::Matrix3d RFromRpy(double roll, double pitch, double yaw)
{
    Eigen::Matrix3d Rx;
    Eigen::Matrix3d Ry;
    Eigen::Matrix3d Rz;
    Rz << std::cos(yaw), -std::sin(yaw), 0.0,
        std::sin(yaw), std::cos(yaw), 0.00,
        0.0, 0.0, 1.0;
    
    Ry << std::cos(pitch), 0.0, std::sin(pitch),
          0.0, 1.0, 0.0,
          -std::sin(pitch), 0.0, std::cos(pitch);
    Rx << 1.0, 0.0, 0.0,
          0.0, std::cos(roll), -std::sin(roll),
          0.0, std::sin(roll), std::cos(roll);
    return Rz * Ry * Rx;
}

std::tuple<double, double, double> RpyFromR(const Eigen::Matrix3d& R) {
    double pitch = std::asin(-R(2,0));
    double yaw;
    double roll;
    if (std::abs(pitch - M_PI / 2) < 1e-6) {
        yaw = 0.0;
        roll = std::atan2(R(0,1), R(1,1));
    }
    else if (std::abs(pitch + M_PI / 2) < 1e-6) {  // Gimbal lock case (θ ≈ -90°)
        yaw = 0;
        roll = -std::atan2(R(0, 1), R(1, 1));
    }
    else {
        yaw = std::atan2(R(1, 0), R(0, 0));  // ψ (yaw)
        roll = std::atan2(R(2, 1), R(2, 2)); // φ (roll)
    }
    return std::make_tuple(roll, pitch, yaw);
}

TEST(TestRotation, TestEulerAngles) {
    double roll = M_PI / 6;
    double pitch = M_PI / 6;
    double yaw = M_PI / 6;
    Eigen::Matrix3d R = RFromRpy(roll, pitch, yaw);
    auto angles = RpyFromR(R);
    double rollApp = std::get<0>(angles);
    double pitchApp = std::get<1>(angles);
    double yawApp = std::get<2>(angles);
    EXPECT_NEAR(roll, rollApp, 1e-2);
    EXPECT_NEAR(pitch, pitchApp, 1e-2);
    EXPECT_NEAR(yaw, yawApp, 1e-2);
}

// Convert Axis-Angle to Quaternion using Rodrigues' formula
Eigen::Quaterniond axisAngle_to_quaternion(const Eigen::Vector3d& axis, double theta) {
    Eigen::Vector3d normalized_axis = axis.normalized();
    double half_theta = theta / 2.0;
    double sin_half_theta = std::sin(half_theta);

    return Eigen::Quaterniond(std::cos(half_theta),
                              sin_half_theta * normalized_axis.x(),
                              sin_half_theta * normalized_axis.y(),
                              sin_half_theta * normalized_axis.z());
}

// Convert Quaternion back to Axis-Angle
std::tuple<Eigen::Vector3d, double> quaternion_to_axisAngle(const Eigen::Quaterniond& q) {
    Eigen::Quaterniond normalized_q = q.normalized();  // Ensure unit quaternion
    double theta = 2.0 * std::acos(normalized_q.w());

    double sin_half_theta = std::sqrt(1.0 - normalized_q.w() * normalized_q.w());
    Eigen::Vector3d axis;
    
    if (sin_half_theta < 1e-6) {  
        // If sin_half_theta is very small, the axis is arbitrary (avoid division by zero)
        axis = Eigen::Vector3d(1.0, 0.0, 0.0);
    } else {
        axis = Eigen::Vector3d(normalized_q.x(), normalized_q.y(), normalized_q.z()) / sin_half_theta;
    }

    return std::make_tuple(axis, theta);
}

// Test conversion from Axis-Angle to Quaternion
TEST(TestRotation, TestAxisAngleToQuaternion) {
    Eigen::Vector3d axis(1.0, 0.0, 0.0);
    double theta = M_PI / 4; // 45 degrees
    Eigen::Quaterniond q = axisAngle_to_quaternion(axis, theta);

    EXPECT_NEAR(q.w(), std::cos(theta / 2), 1e-6);
    EXPECT_NEAR(q.x(), std::sin(theta / 2), 1e-6);
    EXPECT_NEAR(q.y(), 0.0, 1e-6);
    EXPECT_NEAR(q.z(), 0.0, 1e-6);
}

// Test conversion from Quaternion back to Axis-Angle
TEST(TestRotation, TestQuaternionToAxisAngle) {
    Eigen::Quaterniond q(std::cos(M_PI / 6), std::sin(M_PI / 6), 0.0, 0.0);
    auto [axis, theta] = quaternion_to_axisAngle(q);

    EXPECT_NEAR(theta, M_PI / 3, 1e-6);
    EXPECT_NEAR(axis.x(), 1.0, 1e-6);
    EXPECT_NEAR(axis.y(), 0.0, 1e-6);
    EXPECT_NEAR(axis.z(), 0.0, 1e-6);
}

// Test that converting Axis-Angle to Quaternion and back preserves the original rotation
TEST(TestRotation, TestRoundTripAxisAngleQuaternion) {
    for (int i = 0; i < 10; ++i) {
        Eigen::Vector3d axis = Eigen::Vector3d::Random().normalized();
        double theta = (M_PI / 6) + (M_PI / 2) * ((double)std::rand() / RAND_MAX);  // Random angle in [π/6, π]

        Eigen::Quaterniond q = axisAngle_to_quaternion(axis, theta);
        auto [recovered_axis, recovered_theta] = quaternion_to_axisAngle(q);

        EXPECT_NEAR(std::abs(theta), std::abs(recovered_theta), 1e-6);
        EXPECT_TRUE(axis.isApprox(recovered_axis, 1e-6) || axis.isApprox(-recovered_axis, 1e-6));
    }
}