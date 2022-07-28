#include<iostream>

#include<gtsam/base/numericalDerivative.h>
#include<gtsam/geometry/Pose2.h>

gtsam::Vector evaluateError(const gtsam::Pose2& p1, const gtsam::Pose2& p2,
                     boost::optional<gtsam::Matrix&> H1 = boost::none,
                     boost::optional<gtsam::Matrix&> H2 = boost::none) {

    double x_diff = p2.x() - p1.x();
    double y_diff = p2.y() - p1.y();
    Eigen::Vector2d pos_diff(x_diff, y_diff);
    double dist = pos_diff.norm();

    Eigen::Matrix<double, 1, 1> error;
    error << dist;

    if (H1) {
        *H1 = Eigen::Matrix<double, 1, 3>();
        *H1 << -1 * 1/dist * x_diff, -1 * 1/dist * y_diff, 0;
    }

    if (H2) {
        *H2 = Eigen::Matrix<double, 1, 3>();
        *H2 << 1/dist * x_diff, 1/dist * y_diff, 0;
    }

    return error;

}

int main(int argc, char** argv) {
    // Test distance derivative

    double x1 = 0;
    double y1 = 0;
    double theta1 = 0;

    double x2 = 0;
    double y2 = 1.2;
    double theta2 = M_PI/6;

    gtsam::Pose2 pose1(x1, y1, theta1);
    gtsam::Pose2 pose2(x2, y2, theta2);

    gtsam::Matrix expectedH1 = gtsam::numericalDerivative21<gtsam::Vector1,gtsam::Pose2,gtsam::Pose2>(
        boost::function<gtsam::Vector(const gtsam::Pose2&, const gtsam::Pose2&)>(
            boost::bind(&evaluateError, _1, _2, boost::none, boost::none)), pose1, pose2, 1e-5);

    gtsam::Matrix expectedH2 = gtsam::numericalDerivative22<gtsam::Vector1,gtsam::Pose2,gtsam::Pose2>(
        boost::function<gtsam::Vector(const gtsam::Pose2&, const gtsam::Pose2&)>(
            boost::bind(&evaluateError, _1, _2, boost::none, boost::none)), pose1, pose2, 1e-5);


    gtsam::Matrix actualH1;
    gtsam::Matrix actualH2;

    evaluateError(pose1, pose2, actualH1, actualH2);

    gtsam::assert_equal(expectedH1, actualH1, 1e-5);
    gtsam::assert_equal(expectedH2, actualH2, 1e-5);

    return 0;
}