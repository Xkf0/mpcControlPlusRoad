#ifndef ROBOS_SENSOR_MSGS_H_
#define ROBOS_SENSOR_MSGS_H_
#include <vector>
#include "robos/std_msgs.h"
#include "robos/geometry_msgs.h"
namespace robos {
/***************************Imu***************************/
struct Imu{
	Imu(){
		orientation_covariance.fill(0.0);
		angular_velocity_covariance.fill(0.0);
		linear_acceleration_covariance.fill(0.0);
	}
	Header header;
	Quaternion orientation;
	std::array<double, 9>  orientation_covariance;
	Vector3 angular_velocity;
	std::array<double, 9> angular_velocity_covariance;
	Vector3 linear_acceleration;
	std::array<double, 9> linear_acceleration_covariance;
};
static void Stream(std::ostream& s, const std::string& indent, const Imu& v){
	s << indent << "header: " << "\n";
	Stream(s, indent + "  ", v.header);
	s << indent << "orientation: " << v.orientation;
	s << indent << "orientation_covariance:";
	for (size_t i = 0; i < 9; ++i) {
		s <<" " << v.orientation_covariance[i];
	}
	s << "\n";
	s << indent << "angular_velocity: " << v.angular_velocity;
	s << indent << "angular_velocity_covariance:";
	for (size_t i = 0; i < 9; ++i) {
		s << " " << v.angular_velocity_covariance[i];
	}
	s << "\n";
	s << indent << "linear_acceleration: "<<v.linear_acceleration;
	s << indent << "linear_acceleration_covariance:";
	for (size_t i = 0; i < 9; ++i) {
		s << " " << v.linear_acceleration_covariance[i];
	}
	s << "\n";
}
inline std::ostream& operator<<(std::ostream& s, const Imu& v) {
	Stream(s, "", v);
	return s;
}

/***************************Odometry***************************/
struct Odometry{
	Header header;
	std::string child_frame_id;
	PoseWithCovariance pose;
	TwistWithCovariance twist;
};
static void Stream(std::ostream& s, const std::string& indent, const Odometry& v){
	s << indent << "header: " << "\n";
	Stream(s, indent + "  ", v.header);
	s << indent << "child_frame_id: " << v.child_frame_id << "\n";
	s << indent << "pose: " << "\n";
	Stream(s, indent + "  ", v.pose);
	s << indent << "twist: " << "\n";
	Stream(s, indent + "  ", v.twist);
}
inline std::ostream& operator<<(std::ostream& s, const Odometry& v) {
	Stream(s, "", v);
	return s;
}

/***************************LaserScan***************************/
struct LaserScan{
	LaserScan()
		: angle_min(0.f)
		, angle_max(0.f)
		, angle_increment(0.f)
		, time_increment(0.f)
		, scan_time(0.f)
		, range_min(0.f)
		, range_max(0.f) {}
	 Header header;
	 float angle_min;
	 float angle_max;
	 float angle_increment;
	 float time_increment;
	 float scan_time;
	 float range_min;
	 float range_max;
	 std::vector<float> ranges;
	 std::vector<float> intensities;
};
static void Stream(std::ostream& s, const std::string& indent, const LaserScan& v){
	s << indent << "header: " << "\n";
	Stream(s, indent + "  ", v.header);
	s << indent << "angle_min: " << v.angle_min << "\n";
	s << indent << "angle_max: " << v.angle_max << "\n";
	s << indent << "angle_increment: "<< v.angle_increment << "\n";
	s << indent << "time_increment: "<< v.time_increment << "\n";
	s << indent << "scan_time: "<<v.scan_time << "\n";
	s << indent << "range_min: "<< v.range_min << "\n";
	s << indent << "range_max: "<< v.range_max << "\n";
	s << indent << "ranges_size: " << v.ranges.size() << "\n";
	s << indent << "intensities_size: " <<v.intensities.size() << "\n";
}
inline std::ostream& operator<<(std::ostream& s, const LaserScan& v) {
	Stream(s, "", v);
	return s;
}
}  // namespace robos
#endif // !ROBOS_SENSOR_MSGS_H_

