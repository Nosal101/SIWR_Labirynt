#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Point2.h>
#include <fstream>
#include <vector>
#include <cstdio>
#include <iostream>
#include <chrono>
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>

using namespace std::chrono_literals;
using namespace gtsam;
using namespace std;

class UnaryFactor: public NoiseModelFactor1<Pose2> {
  double mx_, my_; ///< X and Y measurements

public:
  UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
    NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

  Vector evaluateError(const Pose2& q,
                       boost::optional<Matrix&> H = boost::none) const
  {
    const Rot2& R = q.rotation();
    if (H) (*H) = (gtsam::Matrix(2, 3) <<
            R.c(), -R.s(), 0.0,
            R.s(), R.c(), 0.0).finished();
    return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
  }
};

class PositionPredictor : public rclcpp::Node
{
public:
  PositionPredictor()
  : Node("position_predictor"),
    odom_prev_x(0.0), odom_prev_y(0.0)
  {
    // Remove old files if they exist
    std::remove("odom.txt");
    std::remove("amcl.txt");
    std::remove("estimate_x.txt");

    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile.keep_last(10);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    // Subscriptions
    amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10, std::bind(&PositionPredictor::amcl_pose_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&PositionPredictor::odom_callback, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", qos_profile, std::bind(&PositionPredictor::map_callback, this, std::placeholders::_1));
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Map received");
    width = msg->info.width;
    height = msg->info.height;
    map_origin_x = msg->info.origin.position.x;
    map_origin_y = msg->info.origin.position.y;
    map_resolution = msg->info.resolution;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto odom_position = msg->pose.pose.position;
    double x = odom_position.x;
    double y = odom_position.y;

    double x_dist = x - odom_prev_x;
    double y_dist = y - odom_prev_y;

    odom_prev_x = x;
    odom_prev_y = y;

    std::ofstream file("odom.txt", std::ios_base::app);
    file << x << ", " << y << std::endl;

    if (sensor1_positions.size() > sensor2_positions.size()) {
      sensor2_positions.emplace_back(x_dist, y_dist);
      sensor3_positions.emplace_back(x, y);
    }
    add_pose_measurement(x, y, "odom");
  }

  void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    auto amcl_position = msg->pose.pose.position;
    double x = amcl_position.x;
    double y = amcl_position.y;

    std::ofstream file("amcl.txt", std::ios_base::app);
    file << x << ", " << y << std::endl;

    sensor1_positions.emplace_back(x, y);
    add_pose_measurement(x, y, "amcl");
  }

  void add_pose_measurement(double x, double y, const std::string &source)
  {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_estimate;

    auto measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));
    int key_1_iterator = 0;
    int key_2_iterator = 1;
    for (size_t i = 0; i < sensor1_positions.size(); i++) {
      auto key1 = Key(i);
      // auto key1 = Key(i);
      // auto key2 = Symbol('z', i);

      initial_estimate.insert(key1, gtsam::Pose2(sensor1_positions[i].first, sensor1_positions[i].second, 0));
      // initial_estimate.insert(key2, gtsam::Point2(sensor3_positions[i].first, sensor3_positions[i].second));

      graph.add(boost::make_shared<UnaryFactor>(key1, sensor1_positions[i].first, sensor1_positions[i].second,unaryNoise));
      // // auto factor = UnaryFactor(key1, sensor1_positions[i].first, sensor1_positions[i].second, measurement_noise);
      // graph.add(boost::make_shared<UnaryFactor>(key2, sensor3_positions[i].first, sensor3_positions[i].second, measurement_noise));

      if (i > 0) {
        auto last_key_x = Key(i-1);
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(last_key_x, key1, gtsam::Pose2(sensor2_positions[i - 1].first, sensor2_positions[i - 1].second, 0), measurement_noise));
      //   graph.add(gtsam::BetweenFactor<gtsam::Point2>(key1, key2, gtsam::Point2(sensor2_positions[i - 1].first, sensor2_positions[i - 1].second), measurement_noise));
      }
      // key_1_iterator += 2;
    }

    // for (size_t i = 0; i < sensor1_positions.size(); i++) {
    //   auto key1 = Key(key_2_iterator);
    //   // auto key1 = Key(i);
    //   // auto key2 = Symbol('z', i);

    //   // initial_estimate.insert(key1, gtsam::Point2(sensor1_positions[i].first, sensor1_positions[i].second));
    //   initial_estimate.insert(key1, gtsam::Point2(sensor3_positions[i].first, sensor3_positions[i].second));

    //   // graph.add(boost::make_shared<UnaryFactor>(key1, sensor1_positions[i].first, sensor1_positions[i].second, measurement_noise));
    //   // // auto factor = UnaryFactor(key1, sensor1_positions[i].first, sensor1_positions[i].second, measurement_noise);
    //   graph.add(boost::make_shared<UnaryFactor>(key1, sensor3_positions[i].first, sensor3_positions[i].second, measurement_noise));

    //   // if (i > 0) {
    //   // //   auto last_key_x = Symbol('x',i-1);
    //   //   auto last_key_z = key_2_iterator - 2;
    //   // //   graph.add(gtsam::BetweenFactor<gtsam::Point2>(last_key_x, key1, gtsam::Point2(sensor2_positions[i - 1].first, sensor2_positions[i - 1].second), measurement_noise));
    //   //   graph.add(gtsam::BetweenFactor<gtsam::Point2>(key1, key2, gtsam::Point2(sensor2_positions[i - 1].first, sensor2_positions[i - 1].second), measurement_noise));
    //   // }
    //   key_2_iterator += 2;
    // }

    // int key_1_iterator_2 = 0;
    // int key_2_iterator_2 = 1;
    // for (size_t i = 0; i < sensor1_positions.size(); i++) {
    //     auto last_key_x = Key(key_1_iterator_2);
    //     auto last_key_z = Key(key_2_iterator_2);
    //     graph.add(gtsam::BetweenFactor<gtsam::Point2>(last_key_x, last_key_z, gtsam::Point2(sensor2_positions[i - 1].first, sensor2_positions[i - 1].second), measurement_noise));
    //     // graph.add(gtsam::BetweenFactor<gtsam::Point2>(key1, key2, gtsam::Point2(sensor2_positions[i - 1].first, sensor2_positions[i - 1].second), measurement_noise));
    //   }
    //   key_1_iterator_2 += 2;
    //   key_2_iterator_2 += 2;

        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate);
    // // gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate.Values());

      auto result = optimizer.optimize();
      result.print();
      if (!result.empty()) {
        auto last_key = Key(sensor1_positions.size()-1);
        std::ofstream file_x("estimate_x.txt", std::ios_base::app);
        file_x << result.at<gtsam::Pose2>(last_key).x() << ", " << result.at<gtsam::Pose2>(last_key).y() << std::endl;
      }
    }



    // if (!result.empty()) {
    //   auto last_key_x = Symbol('x',sensor1_positions.size() - 1);
    //   auto last_key_z = Symbol('z', sensor1_positions.size() - 1);
    //   // auto last_key_x = Symbol(sensor1_positions.size() - 1 + sensor1_positions.size());
    //   // auto last_key_z = Symbol(sensor1_positions.size() - 1 + sensor1_positions.size());

      // std::ofstream file_x("estimate_x.txt", std::ios_base::app);
      // file_x << result.at<gtsam::Point2>(last_key_x).x() << ", " << result.at<gtsam::Point2>(last_key_x).y() << std::endl;
    //   std::ofstream file_z("estimate_z.txt", std::ios_base::app);
    //   file_z << result.at<gtsam::Point2>(last_key_z).x() << ", " << result.at<gtsam::Point2>(last_key_z).y() << std::endl;
    // }
  

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  std::vector<std::pair<double, double>> sensor1_positions;
  std::vector<std::pair<double, double>> sensor2_positions;
  std::vector<std::pair<double, double>> sensor3_positions;

  double odom_prev_x;
  double odom_prev_y;

  int width;
  int height;
  double map_origin_x;
  double map_origin_y;
  double map_resolution;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PositionPredictor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}