#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class LedExtractor : public rclcpp::Node {
public:
    LedExtractor() : Node("led_extractor") {
        
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10, std::bind(&LedExtractor::image_callback, this, std::placeholders::_1));
        
        
        points_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/led_coordinates", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        cv::Mat hsv, mask;
        
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        // Adjust these values based on your exact underwater LED brightness and colour
        cv::Scalar lower_green(40, 100, 100); 
        cv::Scalar upper_green(80, 255, 255);
        cv::inRange(hsv, lower_green, upper_green, mask);

        // Find Contours and extract Centroids
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point2f> centers;
        for (const auto& cnt : contours) {
            cv::Moments m = cv::moments(cnt);
            if (m.m00 > 10) { // Area threshold to filter out small reflections/noise
                float cx = m.m10 / m.m00;
                float cy = m.m01 / m.m00;
                centers.push_back(cv::Point2f(cx, cy));
    

                cv::circle(frame, cv::Point2f(cx, cy), 3, cv::Scalar(255, 0, 0), -1);
            }
        }

        if (centers.size() == 4) {
            // First sort by Y-coordinate to separate Top 2 from Bottom 2
            std::sort(centers.begin(), centers.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
                return a.y < b.y;
            });
            
            // Sort Top pair by X (Left to Right)
            if (centers[0].x > centers[1].x) std::swap(centers[0], centers[1]);
            // Sort Bottom pair by X (Left to Right)
            if (centers[2].x > centers[3].x) std::swap(centers[2], centers[3]);

            auto output_msg = std_msgs::msg::Float32MultiArray();
            for (const auto& point : centers) {
                output_msg.data.push_back(point.x); // u coordinate
                output_msg.data.push_back(point.y); // v coordinate
            }
            
            points_pub_->publish(output_msg);
            
            
            for(size_t i=0; i<centers.size(); ++i) {
                cv::circle(frame, centers[i], 5, cv::Scalar(0, 0, 255), -1);
                cv::putText(frame, std::to_string(i+1), centers[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
            }
            
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "Tracking unstable! Detected %ld LEDs instead of 4", centers.size());
        }

        cv::imshow("Camera Feed & Tracking", frame);
        cv::imshow("HSV Color Mask (Debug)", mask);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr points_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedExtractor>());
    rclcpp::shutdown();
    return 0;
}