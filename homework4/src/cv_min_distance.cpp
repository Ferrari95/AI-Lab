#include "ros/ros.h"
#include "stdlib.h"
#include "string.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "laser_geometry/laser_geometry.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"

class minimumClass {
	protected:
		ros::NodeHandle n_;
		ros::Subscriber sub_;
		sensor_msgs::PointCloud cloud_;
		laser_geometry::LaserProjection projector_;
		tf::TransformListener listener_;
		cv::Mat m_;
		std::string winname_;
		double mH = 500.0;		// altezza finestra
		double mW = 500.0;		// larghezza finestra
		double radius = 10.0;	// raggio del cerchio di visuale da disegnare nella finestra
			
	public:
		minimumClass(ros::NodeHandle n) : n_(n) {
			sub_ = n_.subscribe("base_scan", 10, &minimumClass::minimum, this);
			winname_ = "laser_window";
			cv::namedWindow(winname_, cv::WindowFlags::WINDOW_AUTOSIZE);
		}
		
		void laser2window(geometry_msgs::Point32 pl, cv::Point& pw) {
			double min = !(mH < mW)?mW : mH;
			pl.x = (pl.x * min) / (2.0 * radius);
			pl.y = (pl.y * min) / (2.0 * radius);
			pw.x = (-pl.y) + (mW / 2.0);
			pw.y = (-pl.x) + (mH / 2.0);
		}
		
		void minimum(const sensor_msgs::LaserScan::ConstPtr& msg) {
			// calcolo posizione più vicina
			double min = 0;
			for(int i = 1; i < msg->ranges.size(); i++)
				if(msg->ranges[i] < msg->ranges[min])
					min = i;
			
			// conversione da coordinate polari a cartesiane (rispetto al sistema di riferimento del laser)
			try {
				projector_.transformLaserScanToPointCloud("base_laser_link", *msg, cloud_, listener_);
			}
			catch(tf::TransformException& e) {
				std::cout << e.what();
				return;
			}
			
			// reset immagine
			m_ = cv::Mat(mH, mW, CV_8UC3, cv::Scalar(255, 255, 255));
			
			// stampa su immagine
			cv::Point p1, p2;
			for(int i = 0; i < cloud_.points.size() - 1; i++) {
				laser2window(cloud_.points[i], p1);
				laser2window(cloud_.points[i+1], p2);
				cv::line(m_, p1, p2, cv::Scalar(255, 0, 0), 1, cv::LineTypes::LINE_8, 0);
			}
			
			laser2window(cloud_.points[min], p1);
			cv::circle(m_, p1, (!(mH < mW)?mW : mH) / 40.0, cv::Scalar(0, 0, 255), 1, cv::LineTypes::LINE_8, 0);
			
			cv::imshow(winname_, m_);
			
			cv::waitKey(1.0);
		}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "cv_min_distance");
	ros::NodeHandle n;
	
	minimumClass mc(n);
	
	ros::spin();
	
	return 0;
}
