#ifndef ROBOS_GEOMETRY_MSGS_H_
#define ROBOS_GEOMETRY_MSGS_H_
#include <array>
#include <cmath>
#include "robos/std_msgs.h"

namespace robos {
/***************************Point3_***************************/
template<typename T>
struct Point3_{
	Point3_() :x(0), y(0), z(0) {}
	Point3_(T _x, T _y, T _z) :x(_x), y(_y), z(_z) {}
	T x;
	T y;
	T z;
};
template<typename T>
inline bool operator==(const Point3_<T>& lhs,const Point3_<T>& rhs){
	return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}
template<typename T>
inline bool operator!=(const Point3_<T>& lhs, const Point3_<T>& rhs){
	return !(lhs == rhs);
}
template<typename T>
void Stream(std::ostream& s, const std::string& indent, const Point3_<T>& v) {
	s << indent << "x: " << v.x << ",y: " << v.y << ",z: " << v.z << "\n";
}
template<typename T>
inline std::ostream& operator<<(std::ostream& s,const Point3_<T>& v) {
	Stream(s, "", v);
	return s;
}
using Point3i = Point3_<int>;
using Point3f = Point3_<float>;
using Point3d = Point3_<double>;

/***************************Point2_***************************/
template<typename T>
struct Point2_{
	Point2_() :x(0), y(0) {}
	Point2_(T _x, T _y) :x(_x), y(_y) {}
	T x;
	T y;
};
template<typename T>
inline bool operator==(const Point2_<T>& lhs, const Point2_<T>& rhs){
	return lhs.x == rhs.x && lhs.y == rhs.y;
}
template<typename T>
inline bool operator!=(const Point2_<T>& lhs, const Point2_<T>& rhs){
	return !(lhs == rhs);
}
template<typename T>
void Stream(std::ostream& s, const std::string& indent, const Point2_<T>& v) {
	s << indent << "x: " << v.x << ",y: " << v.y << "\n";
}
template<typename T>
inline std::ostream& operator<<(std::ostream& s, const Point2_<T>& v) {
	Stream(s, "", v);
	return s;
}

using Point2i = Point2_<int>;
using Point2f = Point2_<float>;
using Point2d = Point2_<double>;

/***************************Vector3***************************/
struct Vector3 {
	Vector3() :x(0.0), y(0.0), z(0.0) {}
	Vector3(double _x, double _y, double _z)
		:x(_x), y(_y), z(_z) {}
	double x;
	double y;
	double z;
};
inline bool operator==(const Vector3& lhs, const Vector3& rhs) {
	return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}
inline bool operator!=(const Vector3& lhs, const Vector3& rhs) {
	return !(lhs == rhs);
}
static void Stream(std::ostream& s, const std::string& indent, const Vector3& v) {
	s << indent << "x: " << v.x << ",y: " << v.y << ",z: " << v.z << "\n";
}
inline std::ostream& operator<<(std::ostream& s, const Vector3& v) {
	Stream(s, "", v);
	return s;
}

/***************************Quaternion***************************/
struct Quaternion{
	Quaternion():x(0.0), y(0.0), z(0.0), w(1.0) {}
	Quaternion(double _x, double _y, double _z, double _w)
		:x(_x), y(_y), z(_z), w(_w) {}
	double x;
	double y;
	double z;
	double w;
};
inline bool operator==(const Quaternion& lhs,const Quaternion& rhs){
	return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w;
}
inline bool operator!=(const Quaternion& lhs, const Quaternion& rhs){
	return !(lhs == rhs);
}
static void Stream(std::ostream& s, const std::string& indent, const Quaternion& v) {
	s << indent << "x: " << v.x << ",y: " << v.y << ",z: " << v.z << ",w: " << v.w << "\n";
}
inline std::ostream& operator<<(std::ostream& s, const Quaternion& v) {
	Stream(s, "", v);
	return s;
}

/***************************Pose2D***************************/
struct Pose2D {
	Pose2D() :x(0.0), y(0.0), theta(0.0) {}
	Pose2D(double _x, double _y, double _theta)
		:x(_x), y(_y), theta(_theta) {}
	double x;
	double y;
	double theta;
};
inline bool operator==(const Pose2D& lhs, const Pose2D& rhs) {
	return lhs.x == rhs.x && lhs.y == rhs.y && lhs.theta == rhs.theta;
}
inline bool operator!=(const Pose2D& lhs, const Pose2D& rhs) {
	return !(lhs == rhs);
}
static void Stream(std::ostream& s, const std::string& indent, const Pose2D& v) {
	s << indent << "x: " << v.x << ",y: " << v.y << ",theta: " << v.theta << "\n";
}
inline std::ostream& operator<<(std::ostream& s, const Pose2D& v) {
	Stream(s, "", v);
	return s;
}

/***************************Pose***************************/
struct Pose{
	Pose() = default;
	Pose(const Point3d& _position,const Quaternion& _orientation)
		:position(_position), orientation(_orientation) {}
	Point3d position;
	Quaternion orientation;
};
inline bool operator==(const Pose& lhs, const Pose& rhs) {
	return lhs.position == rhs.position && lhs.orientation == rhs.orientation;
}
inline bool operator!=(const Pose& lhs, const Pose& rhs) {
	return !(lhs == rhs);
}
static void Stream(std::ostream& s, const std::string& indent, const Pose& v) {
	s << indent << "position: " << v.position;
	s << indent << "orientation: " << v.orientation;
}
inline std::ostream& operator<<(std::ostream& s, const Pose& v) {
	Stream(s, "", v);
	return s;
}

/***************************PoseStamped2D***************************/
struct PoseStamped2D {
	Header header;
	Pose2D pose;
};
inline bool operator==(const PoseStamped2D& lhs, const PoseStamped2D& rhs) {
	return lhs.header == rhs.header && lhs.pose == rhs.pose;
}
inline bool operator!=(const PoseStamped2D& lhs, const PoseStamped2D& rhs) {
	return !(lhs == rhs);
}
static void Stream(std::ostream& s, const std::string& indent, const PoseStamped2D& v) {
	s << indent << "header: " << "\n";
	Stream(s, indent + "  ", v.header);
	s << indent << "pose: " << v.pose;
}
inline std::ostream& operator<<(std::ostream& s, const PoseStamped2D& v) {
	Stream(s, "", v);
	return s;
}

/***************************PoseStamped***************************/
struct PoseStamped {
	Header header;
	Pose pose;
};
inline bool operator==(const PoseStamped& lhs, const PoseStamped& rhs) {
	return lhs.header == rhs.header && lhs.pose == rhs.pose;
}
inline bool operator!=(const PoseStamped& lhs, const PoseStamped& rhs) {
	return !(lhs == rhs);
}
static void Stream(std::ostream& s, const std::string& indent, const PoseStamped& v) {
	s << indent << "header: " << "\n";
	Stream(s, indent + "  ", v.header);
	s << indent << "pose: " << "\n";
	Stream(s, indent + "  ", v.pose);
}
inline std::ostream& operator<<(std::ostream& s, const PoseStamped& v) {
	Stream(s, "", v);
	return s;
}

/***************************PoseWithCovariance***************************/
struct PoseWithCovariance{
	PoseWithCovariance() {
		covariance.fill(0.0);
	}
	Pose pose;
	std::array<double, 36> covariance;
};
inline bool operator==(const PoseWithCovariance& lhs, const PoseWithCovariance& rhs){
	return lhs.pose == rhs.pose &&
		lhs.covariance == rhs.covariance;
}

inline bool operator!=(const PoseWithCovariance& lhs, const PoseWithCovariance& rhs){
	return !(lhs == rhs);
}
static void Stream(std::ostream& s, const std::string& indent, const PoseWithCovariance& v) {
	s << indent << "pose: " << "\n";
	Stream(s, indent + "  ", v.pose);
	s << indent << "covariance: " << "\n";
	for (size_t i = 0; i < 6; ++i){
		s << indent;
		for (size_t j = 0; j < 6; ++j) {
			s << "  " << v.covariance[i * 6 + j];
		}
		s << "\n";
	}
}
inline std::ostream& operator<<(std::ostream& s, const PoseWithCovariance& v) {
	Stream(s, "", v);
	return s;
}

/***************************Transform***************************/
struct Transform{
	Transform() = default;
	Transform(const Vector3& _translation, const Quaternion& _rotation)
		:translation(_translation), rotation(_rotation) {}
	Vector3 translation;
	Quaternion rotation;
};
inline bool operator==(const Transform& lhs, const Transform& rhs) {
	return lhs.translation == rhs.translation && lhs.rotation == rhs.rotation;
}
inline bool operator!=(const Transform& lhs, const Transform& rhs) {
	return !(lhs == rhs);
}
static void Stream(std::ostream& s, const std::string& indent, const Transform& v) {
	s << indent << "translation: " << v.translation;
	s << indent << "rotation: " << v.rotation;
}
inline std::ostream& operator<<(std::ostream& s, const Transform& v) {
	Stream(s, "", v);
	return s;
}
/***************************TransformStamped***************************/
struct TransformStamped{
	Header header;
	std::string child_frame_id;
	Transform transform;
};
inline bool operator==(const TransformStamped& lhs, const TransformStamped& rhs) {
	return lhs.header == rhs.header
		&& lhs.child_frame_id == rhs.child_frame_id
		&& lhs.transform == rhs.transform;
}
inline bool operator!=(const TransformStamped& lhs, const TransformStamped& rhs) {
	return !(lhs == rhs);
}
static void Stream(std::ostream& s, const std::string& indent, const TransformStamped& v) {
	s << indent << "header: " << "\n";
	Stream(s, indent + "  ", v.header);
	s << indent << "child_frame_id: " <<v.child_frame_id<< "\n";
	s << indent << "transform: " << "\n";
	Stream(s, indent + "  ", v.transform);
}
inline std::ostream& operator<<(std::ostream& s, const TransformStamped& v) {
	Stream(s, "", v);
	return s;
}

/***************************Twist***************************/
struct Twist{
	Twist() = default;
	Twist(const Vector3& _linear,const Vector3& _angular)
		:linear(_linear), angular(_angular) {}
	Vector3 linear;
	Vector3 angular;
};
inline bool operator==(const Twist& lhs, const Twist& rhs) {
	return lhs.linear == rhs.linear && lhs.angular == rhs.angular;
}
inline bool operator!=(const Twist& lhs, const Twist& rhs) {
	return !(lhs == rhs);
}
static void Stream(std::ostream& s, const std::string& indent, const Twist& v) {
	s << indent << "linear: " << v.linear;
	s << indent << "angular: " << v.angular;
}
inline std::ostream& operator<<(std::ostream& s, const Twist& v) {
	Stream(s, "", v);
	return s;
}

/***************************TwistWithCovariance***************************/
struct TwistWithCovariance{
	TwistWithCovariance() {
		covariance.fill(0.0);
	}
	Twist twist;
	std::array<double, 36> covariance;
};
inline bool operator==(const TwistWithCovariance& lhs, const TwistWithCovariance& rhs) {
	return lhs.twist == rhs.twist &&
		lhs.covariance == rhs.covariance;
}
inline bool operator!=(const TwistWithCovariance& lhs, const TwistWithCovariance& rhs) {
	return !(lhs == rhs);
}
static void Stream(std::ostream& s, const std::string& indent, const TwistWithCovariance& v) {
	s << indent << "twist: " << "\n";
	Stream(s, indent + "  ", v.twist);
	s << indent << "covariance: " << "\n";
	for (size_t i = 0; i < 6; ++i) {
		s << indent;
		for (size_t j = 0; j < 6; ++j) {
			s << "  " << v.covariance[i * 6 + j];
		}
		s << "\n";
	}
}
inline std::ostream& operator<<(std::ostream& s, const TwistWithCovariance& v) {
	Stream(s, "", v);
	return s;
}

/***************************Fuctions***************************/
static Quaternion CreateQuaternionMsg(
	const double& roll, const double& pitch, const double& yaw){
	double halfYaw = yaw * 0.5;
	double halfPitch = pitch * 0.5;
	double halfRoll = roll * 0.5;
	double cosYaw = cos(halfYaw);
	double sinYaw = sin(halfYaw);
	double cosPitch = cos(halfPitch);
	double sinPitch = sin(halfPitch);
	double cosRoll = cos(halfRoll);
	double sinRoll = sin(halfRoll);
	Quaternion q(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
		cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
		cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
		cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //formerly yzx
	return q;
}

static Quaternion CreateQuaternionMsg(const double& yaw) {
	return CreateQuaternionMsg(0., 0., yaw);
}

static void GetRPY(const Quaternion& q,
	double& roll, double& pitch, double& yaw) {

	double sqw;
	double sqx;
	double sqy;
	double sqz;

	sqx = q.x * q.x;
	sqy = q.y * q.y;
	sqz = q.z * q.z;
	sqw = q.w * q.w;

	// Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
	double sarg = -2 * (q.x * q.z - q.w * q.y) / (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */
	if (sarg <= -0.99999) {
		pitch = -0.5 * M_PI;
		roll = 0;
		yaw = -2 * atan2(q.y, q.x);
	}
	else if (sarg >= 0.99999) {
		pitch = 0.5 * M_PI;
		roll = 0;
		yaw = 2 * atan2(q.y, q.x);
	}
	else {
		pitch = asin(sarg);
		roll = atan2(2 * (q.y * q.z + q.w * q.x), sqw - sqx - sqy + sqz);
		yaw = atan2(2 * (q.x * q.y + q.w * q.z), sqw + sqx - sqy - sqz);
	}
}

static double GetYaw(const Quaternion& q) {
	double roll, pitch, yaw;
	GetRPY(q, roll, pitch, yaw);
	return yaw;
}

static Pose Embed3D(const Pose2D& pose) {
	Pose result;
	result.position.x = pose.x;
	result.position.y = pose.y;
	result.position.z = 0.0;
	result.orientation = CreateQuaternionMsg(pose.theta);
	return result;
}

static PoseStamped Embed3D(const PoseStamped2D& pose) {
	PoseStamped result;
	result.header = pose.header;
	result.pose = Embed3D(pose.pose);
	return result;
}

static Pose2D Project2D(const Pose& pose) {
	Pose2D result;
	result.x = pose.position.x;
	result.y = pose.position.y;
	result.theta = GetYaw(pose.orientation);
	return result;
}

static PoseStamped2D Project2D(const PoseStamped& pose) {
	PoseStamped2D result;
	result.header = pose.header;
	result.pose = Project2D(pose.pose);
	return result;
}
}// namespace robos
#endif // !ROBOS_GEOMETRY_MSGS_H_
