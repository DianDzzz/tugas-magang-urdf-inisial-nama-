#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <cmath>

std::pair<double, double> inverseKinematics(double x, double y, double l1, double l2) {
    double D = (x*x + y*y - l1*l1 - l2*l2) / (2 * l1 * l2);
    double theta2 = atan2(sqrt(1 - D*D), D); 
    double theta1 = atan2(y, x) - atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));
    return std::make_pair(theta1, theta2);
}

std::vector<std::pair<double, double>> spellNameTrajectory(const std::string& name) {
    std::vector<std::pair<double, double>> trajectory;
    std::map<char, std::vector<std::pair<double, double>>> namePoints = {
        {'D', {{1, -1}, {1.5, -0.7}, {1.5, -0.7}, {1, 1}}},
        {'I', {{1, -1}, {1, 1}}},
        {'D', {{1, -1}, {1.5, -0.7}, {1.5, -0.7}, {1, 1}}},
        {'A', {{1, -1}, {1.5, 1.3}, {1, 1.5}}},
        {'N', {{1, -1}, {1.5, 2}, {1.5, 1}}}
    };

    for (char letter : name) {
        if (namePoints.find(letter) != namePoints.end()) {
            trajectory.insert(trajectory.end(), namePoints[letter].begin(), namePoints[letter].end());
        }
    }
    return trajectory;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;

    ros::Publisher joint1_pub = nh.advertise<std_msgs::Float64>("/joint1_position_controller/command", 10);
    ros::Publisher joint2_pub = nh.advertise<std_msgs::Float64>("/joint2_position_controller/command", 10);

    ros::Rate rate(10);

    double l1 = 0.03, l2 = 0.03;

    std::vector<std::pair<double, double>> trajectory_points = spellNameTrajectory("DIDAN");

    for (const auto& point : trajectory_points) {
        double x = point.first;
        double y = point.second;

        auto angles = inverseKinematics(x, y, l1, l2);
        double theta1 = angles.first;
        double theta2 = angles.second;

        std_msgs::Float64 shoulder_msg;
        shoulder_msg.data = theta1;
        shoulder_pub.publish(shoulder_msg);

        std_msgs::Float64 elbow_msg;
        elbow_msg.data = theta2;
        elbow_pub.publish(elbow_msg);

        rate.sleep();
    }

    return 0;
}
