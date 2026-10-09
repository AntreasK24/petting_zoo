// ROS 2 node that detects red and green apples in a compressed image stream
// using classical colour + shape analysis (no neural network).
//
// Pipeline per frame:
//   1. decode -> optional blur -> HSV
//   2. threshold a colour mask (red wraps hue 0, so two ranges; green is one)
//   3. morphological open/close to clean the mask
//   4. find contours, keep the round, solid, large-enough ones (apple shape)
//   5. publish a bounding box per surviving contour
//
// Red and green apples are classified separately: each colour has its own
// output topic ("red_apple" / "green_apple"). Everything is tunable from
// config/apple_detector.yaml.

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>

using sensor_msgs::msg::CompressedImage;
using vision_msgs::msg::BoundingBox2D;
using vision_msgs::msg::BoundingBox2DArray;

// One apple colour: its HSV range(s) and output publisher.
struct ColourClass
{
  std::string name;
  std::string label;
  rclcpp::Publisher<BoundingBox2DArray>::SharedPtr pub;
  std::vector<std::pair<cv::Scalar, cv::Scalar>> ranges;  // HSV lower/upper pairs
};

class AppleDetector : public rclcpp::Node
{
public:
  AppleDetector()
  : Node("apple_detector")
  {
    // ---- input -----------------------------------------------------------
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/camera/image_raw/compressed");
    const int queue_size = declare_parameter<int>("queue_size", 1);
    const double max_rate = declare_parameter<double>("max_rate", 0.0);
    min_period_ = (max_rate > 0.0) ? 1.0 / max_rate : 0.0;

    // ---- shape filtering -------------------------------------------------
    min_area_ = declare_parameter<int>("min_area", 500);
    max_area_ = declare_parameter<int>("max_area", 0);
    min_circularity_ = declare_parameter<double>("min_circularity", 0.6);
    min_solidity_ = declare_parameter<double>("min_solidity", 0.85);
    blur_ksize_ = declare_parameter<int>("blur_ksize", 5);
    const int morph = declare_parameter<int>("morph_ksize", 5);
    morph_kernel_ = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(std::max(1, morph), std::max(1, morph)));

    // ---- colour classes --------------------------------------------------
    // Exactly two fixed classes: red and green apples. Only their HSV
    // thresholds are tunable from YAML (<colour>.hue_ranges / sat_min /
    // val_min); topic and label are fixed.
    setup_colour("red", "red_apple", {0, 10, 170, 179}, 120, 60);
    setup_colour("green", "green_apple", {28, 95}, 40, 30);
    if (classes_.empty()) {
      RCLCPP_FATAL(get_logger(), "No apple colours configured; shutting down.");
      throw std::runtime_error("no colours configured");
    }

    sub_ = create_subscription<CompressedImage>(
      image_topic_, queue_size,
      std::bind(&AppleDetector::image_cb, this, std::placeholders::_1));

    std::string names;
    for (const auto & c : classes_) {names += (names.empty() ? "" : ", ") + c.name;}
    RCLCPP_INFO(
      get_logger(),
      "apple_detector ready: colours=[%s], min_area=%d, min_circularity=%.2f, "
      "subscribing to '%s'",
      names.c_str(), min_area_, min_circularity_, image_topic_.c_str());
  }

private:
  // ---------------------------------------------------------------------
  // Set up one colour from easy parameters: a list of hue ranges plus a
  // saturation/value floor (max S/V are always 255). hue_ranges is a flat
  // list of [lo, hi, lo, hi, ...] pairs, so red can span 0-10 AND 170-179.
  void setup_colour(
    const std::string & name, const std::string & label,
    const std::vector<int64_t> & def_hue, int def_sat_min, int def_val_min)
  {
    const auto hue = declare_parameter<std::vector<int64_t>>(name + ".hue_ranges", def_hue);
    const int sat_min = declare_parameter<int>(name + ".sat_min", def_sat_min);
    const int val_min = declare_parameter<int>(name + ".val_min", def_val_min);

    if (hue.size() < 2 || hue.size() % 2 != 0) {
      RCLCPP_ERROR(
        get_logger(), "%s.hue_ranges must be pairs [lo,hi,...]; skipping '%s'",
        name.c_str(), name.c_str());
      return;
    }

    ColourClass c;
    c.name = name;
    c.label = label;
    for (size_t i = 0; i + 1 < hue.size(); i += 2) {
      c.ranges.emplace_back(
        cv::Scalar(hue[i], sat_min, val_min),
        cv::Scalar(hue[i + 1], 255, 255));
    }
    const std::string topic = "detections/apples/" + name;
    c.pub = create_publisher<BoundingBox2DArray>(topic, 10);
    classes_.push_back(c);
    RCLCPP_INFO(
      get_logger(), "%s -> '%s' (label '%s', %zu hue range(s), S>=%d V>=%d)",
      name.c_str(), topic.c_str(), label.c_str(), c.ranges.size(), sat_min, val_min);
  }

  // ---------------------------------------------------------------------
  static double steady_seconds()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void image_cb(const CompressedImage::SharedPtr msg)
  {
    // Rate limit: drop the frame before the costly decode.
    if (min_period_ > 0.0) {
      const double now = steady_seconds();
      if (now - last_proc_ < min_period_) {return;}
      last_proc_ = now;
    }

    cv::Mat raw(1, static_cast<int>(msg->data.size()), CV_8UC1,
      const_cast<unsigned char *>(msg->data.data()));
    const cv::Mat frame = cv::imdecode(raw, cv::IMREAD_COLOR);
    if (frame.empty()) {
      RCLCPP_WARN(get_logger(), "failed to decode image");
      return;
    }

    cv::Mat proc = frame;
    if (blur_ksize_ >= 3) {
      const int k = blur_ksize_ | 1;  // must be odd
      cv::GaussianBlur(frame, proc, cv::Size(k, k), 0);
    }
    cv::Mat hsv;
    cv::cvtColor(proc, hsv, cv::COLOR_BGR2HSV);

    for (auto & cls : classes_) {
      cv::Mat mask;
      for (const auto & r : cls.ranges) {
        cv::Mat m;
        cv::inRange(hsv, r.first, r.second, m);
        if (mask.empty()) {mask = m;} else {cv::bitwise_or(mask, m, mask);}
      }
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, morph_kernel_);
      cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, morph_kernel_);

      BoundingBox2DArray out;
      out.header = msg->header;
      for (const auto & r : contours_to_boxes(mask)) {
        BoundingBox2D box;
        // vision_msgs boxes are center-based; convert from OpenCV's top-left rect.
        box.center.position.x = r.x + r.width / 2.0;
        box.center.position.y = r.y + r.height / 2.0;
        box.size_x = r.width;
        box.size_y = r.height;
        out.boxes.push_back(box);
      }
      cls.pub->publish(out);
    }
  }

  // ---------------------------------------------------------------------
  std::vector<cv::Rect> contours_to_boxes(const cv::Mat & mask)
  {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Rect> boxes;
    for (const auto & c : contours) {
      const double area = cv::contourArea(c);
      if (area < min_area_) {continue;}
      if (max_area_ > 0 && area > max_area_) {continue;}

      const double perimeter = cv::arcLength(c, true);
      if (perimeter <= 0.0) {continue;}
      // Circularity: 1.0 for a perfect circle; apples are roundish.
      const double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
      if (circularity < min_circularity_) {continue;}
      // Solidity: contour area / convex-hull area; high for a solid blob.
      std::vector<cv::Point> hull;
      cv::convexHull(c, hull);
      const double hull_area = cv::contourArea(hull);
      const double solidity = (hull_area > 0.0) ? area / hull_area : 0.0;
      if (solidity < min_solidity_) {continue;}

      boxes.push_back(cv::boundingRect(c));
    }
    return boxes;
  }

  // ---- members ---------------------------------------------------------
  std::string image_topic_;
  double min_period_{0.0};
  double last_proc_{0.0};

  int min_area_{500};
  int max_area_{0};
  double min_circularity_{0.6};
  double min_solidity_{0.85};
  int blur_ksize_{5};
  cv::Mat morph_kernel_;

  std::vector<ColourClass> classes_;
  rclcpp::Subscription<CompressedImage>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AppleDetector>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("apple_detector"), "startup failed: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
