// Copyright 2025 Hency
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     https://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <array>
#include <vector>
#include <cmath>
#include <exception>

/* ========================= Constants ========================= */
constexpr double  kMapResolution      = 0.05;     // m/cell
constexpr double  kMapWidthMeters     = 20.0;     // map covers ±10 m
constexpr int     kMatSide            = 7;        // 7×7 mask
constexpr int     kMatLen             = kMatSide * kMatSide;
constexpr double  kControllerFreq     = 10.0;     // Hz (default)
constexpr double  kDefaultScanGain    = 0.04;
constexpr double  kRangeLimGain       = 2.0;
constexpr double  kAveLimGain         = 1.0;
constexpr double  kGmapThreshold      = 3.0;
constexpr double  kGmapDotRadius      = 3.0;
constexpr int     kObstacleRadius     = 9;        // search window for path optimisation

/* 7×7 DoG‑like kernel，用于锥桶检测 */
static const std::array<int, kMatLen> kDotKernel = {
     -5, -5, -5, -5, -5, -5, -5,
     -5, -2, -2, -2, -2, -2, -5,
     -5, -2,  2,  2,  2, -2, -5,
     -5, -2,  2,  1,  2, -2, -5,
     -5, -2,  2,  2,  2, -2, -5,
     -5, -2, -2, -2, -2, -2, -5,
     -5, -5, -5, -5, -5, -5, -5};

/* ====================== Mapping class ======================= */
class Mapping
{
public:
  Mapping();
  void spin() { ros::spin(); }

private:
  /* === Core helpers === */
  static bool dotDetect(const nav_msgs::OccupancyGrid& map,
                        double threshold, int index, int radius);
  static bool dotDetectByMat(const nav_msgs::OccupancyGrid& map,
                             int threshold, int index);
  static void copyWindow(std::vector<int>& buf,
                         const nav_msgs::OccupancyGrid& map,
                         int index, int side);

  void createMap();
  void fusionGmap();
  void optimisePath();

  /* === ROS callbacks === */
  void gmapCB (const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void scanCB (const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCB (const nav_msgs::Odometry::ConstPtr& msg);
  void pathCB (const nav_msgs::Path::ConstPtr& msg);
  void timerCB(const ros::TimerEvent&);

  /* === ROS handles === */
  ros::NodeHandle nh_;
  ros::Timer      timer_;

  ros::Publisher  map_pub_;
  ros::Publisher  gmap_pub_;
  ros::Publisher  path_pub_;
  ros::Publisher  bias_pub_;

  ros::Subscriber scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber gmapping_sub_;
  ros::Subscriber path_sub_;

  /* === Runtime data === */
  nav_msgs::OccupancyGrid map_;
  nav_msgs::OccupancyGrid gmap_;
  nav_msgs::OccupancyGrid gmap_copy_;

  nav_msgs::Odometry      odom_;
  sensor_msgs::LaserScan  scan_;
  nav_msgs::Path          path_;
  nav_msgs::Path          map_path_;
  nav_msgs::Path          optimised_path_;

  /* === Parameters === */
  double controller_freq_ = kControllerFreq;
  double scan_gain_       = kDefaultScanGain;
  double range_lim_gain_  = kRangeLimGain;
  double ave_lim_gain_    = kAveLimGain;
  double gmap_thr_        = kGmapThreshold;
  double gmap_dot_radius_ = kGmapDotRadius;

  bool   got_path_   = false;
  bool   gmap_fused_ = false;
};

/* =================== Implementation =================== */
Mapping::Mapping()
{
  // ── retrieve parameters ──
  ros::NodeHandle pnh("~");
  pnh.param("controllerFreq", controller_freq_,  controller_freq_);
  pnh.param("scan_gain",      scan_gain_,        scan_gain_);
  pnh.param("range_lim_gain", range_lim_gain_,   range_lim_gain_);
  pnh.param("ave_lim_gain",   ave_lim_gain_,     ave_lim_gain_);
  pnh.param("gmap_thr",       gmap_thr_,         gmap_thr_);
  pnh.param("gmap_dot_radius",gmap_dot_radius_,  gmap_dot_radius_);

  // ── publishers ──
  map_pub_  = nh_.advertise<nav_msgs::OccupancyGrid>("omapping/map", 1, true);
  gmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("gmapping/map", 1, true);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/optimised/path", 1, true);
  bias_pub_ = nh_.advertise<std_msgs::Float32>("/scan/bias", 1, true);

  // ── subscribers ──
  scan_sub_    = nh_.subscribe("scan", 1, &Mapping::scanCB, this);
  gmapping_sub_= nh_.subscribe("map", 1, &Mapping::gmapCB, this);
  path_sub_    = nh_.subscribe("/recorded/path", 1, &Mapping::pathCB, this);
  odom_sub_    = nh_.subscribe("/odometry/filtered", 1, &Mapping::odomCB, this);

  // ── local occupancy map init ──
  map_.header.frame_id   = "map";
  map_.info.resolution   = kMapResolution;
  map_.info.width        = kMapWidthMeters / kMapResolution;
  map_.info.height       = kMapWidthMeters / kMapResolution;
  map_.info.origin.position.x = -kMapWidthMeters / 2.0;
  map_.info.origin.position.y = -kMapWidthMeters / 2.0;
  map_.data.assign(map_.info.width * map_.info.height, 0);

  // ── timer ──
  timer_ = nh_.createTimer(ros::Duration(1.0 / controller_freq_),
                           &Mapping::timerCB, this);
}

/* ---------- static helpers ---------- */
bool Mapping::dotDetect(const nav_msgs::OccupancyGrid& map,
                        double threshold, int index, int radius)
{
  double ave = 0.0;
  const int w = map.info.width;
  const double denom = radius * radius;
  for (int dy = -(radius-1)/2; dy <= (radius-1)/2; ++dy)
    for (int dx = -(radius-1)/2; dx <= (radius-1)/2; ++dx)
      ave += map.data[index + dx + dy * w] / denom;
  return ave > threshold;
}

void Mapping::copyWindow(std::vector<int>& buf,
                         const nav_msgs::OccupancyGrid& map,
                         int index, int side)
{
  buf.resize(side*side);
  const int w = map.info.width;
  int idx = 0;
  for (int dy = -(side-1)/2; dy <= (side-1)/2; ++dy)
    for (int dx = -(side-1)/2; dx <= (side-1)/2; ++dx)
      buf[idx++] = map.data[index + dx + dy * w];
}

bool Mapping::dotDetectByMat(const nav_msgs::OccupancyGrid& map,
                             int threshold, int index)
{
  std::vector<int> win;
  copyWindow(win, map, index, kMatSide);
  int score = 0;
  for (int i = 0; i < kMatLen; ++i) score += win[i] * kDotKernel[i];
  return score > threshold;
}

/* ---------- ROS callbacks ---------- */
void Mapping::gmapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  if (got_path_ && !gmap_fused_) {
    gmap_      = *msg;
    gmap_copy_ = *msg;
    fusionGmap();
    gmap_pub_.publish(gmap_copy_);
  }
}

void Mapping::scanCB(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  scan_ = *msg;
  createMap();
}

void Mapping::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_ = *msg;
}

void Mapping::pathCB(const nav_msgs::Path::ConstPtr& msg)
{
  path_ = *msg;
  optimisePath();
  got_path_ = true;
}

void Mapping::timerCB(const ros::TimerEvent&)
{
  /* reserved for future periodic tasks */
}

/* ---------- fusion, mapping, path ---------- */
void Mapping::fusionGmap()
{
  const int w = gmap_.info.width;
  const int h = gmap_.info.height;
  const int len = w * h;
  for (int idx = 0; idx < len; ++idx) {
    if (dotDetectByMat(gmap_, gmap_thr_, idx)) {
      gmap_copy_.data[idx] = 150; // mark cone
    }
  }
  gmap_fused_ = true;
}

void Mapping::createMap()
{
  /* simplified & cleaned version: only accumulate scan hits into map_ */
  const double yaw = tf::getYaw(odom_.pose.pose.orientation);
  const double car_x = odom_.pose.pose.position.x;
  const double car_y = odom_.pose.pose.position.y;
  const int w = map_.info.width;
  const int h = map_.info.height;

  for (size_t i = 0; i < scan_.ranges.size(); ++i) {
    const double range = scan_.ranges[i];
    if (range < 0.3 || range > 10.0) continue;

    const double angle = yaw + 3.1415926535 + scan_.angle_min + i * scan_.angle_increment;
    const double gx = range * std::cos(angle) + car_x;
    const double gy = range * std::sin(angle) + car_y;
    const int x_idx = static_cast<int>(gx / map_.info.resolution) + w / 2;
    const int y_idx = static_cast<int>(gy / map_.info.resolution) + h / 2;
    if (x_idx < 0 || x_idx >= w || y_idx < 0 || y_idx >= h) continue;
    const int map_idx = x_idx + y_idx * w;
    map_.data[map_idx] = std::min(100, map_.data[map_idx] + 1);
  }

  map_.header.stamp = ros::Time::now();
  map_pub_.publish(map_);
}

void Mapping::optimisePath()
{
  if (path_.poses.empty()) return;

  optimised_path_.poses.clear();
  optimised_path_.header.frame_id = "odom";
  optimised_path_.header.stamp    = ros::Time::now();

  const int size = static_cast<int>(path_.poses.size());
  for (int i = 0; i < size; ++i) {
    geometry_msgs::PoseStamped p = path_.poses[i];
    // simple moving average smoothing (±11)
    const int window = 11;
    int cnt = 0;
    double sx = 0, sy = 0;
    for (int j = -window; j <= window; ++j) {
      int idx = std::clamp(i + j, 0, size - 1);
      sx += path_.poses[idx].pose.position.x;
      sy += path_.poses[idx].pose.position.y;
      ++cnt;
    }
    p.pose.position.x = sx / cnt;
    p.pose.position.y = sy / cnt;
    optimised_path_.poses.push_back(std::move(p));
  }
  path_pub_.publish(optimised_path_);
}

/* =================== main =================== */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "mapping");
  Mapping node;
  node.spin();
  return 0;
}
