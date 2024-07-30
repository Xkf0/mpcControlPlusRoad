#ifndef ROBOS_NAVI_MSGS_H_
#define ROBOS_NAVI_MSGS_H_
#include <cfloat>
#include <vector>
#include "robos/std_msgs.h"
#include "robos/geometry_msgs.h"
namespace robos{
/***************************PoseInfo2D***************************/
struct PoseInfo2D{
	PoseInfo2D() :mode(0), confidence(0.f) {}
	Header header;
	Pose2D pose;
	uint8_t mode;
	float confidence;
};
static void Stream(std::ostream& s, const std::string& indent, const PoseInfo2D& v) {
	s << indent << "header: " << "\n";
	Stream(s, indent + "  ", v.header);
	s << indent << "pose: " << v.pose;
	s << indent << "mode: " << static_cast<uint32_t>(v.mode) << "\n";
	s << indent << "confidence: " << v.confidence << "\n";
}
inline std::ostream& operator<<(std::ostream& s, const PoseInfo2D& v) {
	Stream(s, "", v);
	return s;
}

/***************************PointCloud2D***************************/
struct PointCloud2D{
	Header header;
	std::vector<Point2f> points;
};
static void Stream(std::ostream& s, const std::string& indent, const PointCloud2D& v) {
	s << indent << "header: " << "\n";
	Stream(s, indent + "  ", v.header);
	s << indent << "points_size: " << v.points.size() << "\n";
}
inline std::ostream& operator<<(std::ostream& s, const PointCloud2D& v) {
	Stream(s, "", v);
	return s;
}

/***************************MapMetaData***************************/
struct MapMetaData{
	MapMetaData() :resolution(0.f), width(0), height(0) {}
	robos::Time map_load_time;
	float resolution;
	uint32_t width;
	uint32_t height;
	Pose2D origin;
};
static void Stream(std::ostream& s, const std::string& indent, const MapMetaData& v) {
	s << indent << "map_load_time: " << v.map_load_time << "\n";
	s << indent << "resolution: " << v.resolution << "\n";
	s << indent << "width: " << v.width << "\n";
	s << indent << "height: " << v.height << "\n";
	s << indent << "origin: " << v.origin;
}
inline std::ostream& operator<<(std::ostream& s, const MapMetaData& v) {
	Stream(s, "", v);
	return s;
}

/***************************OccupancyGrid***************************/
struct OccupancyGrid{
	Header header;
	MapMetaData info;
	std::vector<int8_t> data;
};
static void Stream(std::ostream& s, const std::string& indent, const OccupancyGrid& v) {
	s << indent << "header: " << "\n";
	Stream(s, indent + "  ", v.header);
	s << indent << "info: " << "\n";
	Stream(s, indent + "  ", v.info);
	s << indent << "data_size: " << v.data.size() << "\n";
}
inline std::ostream& operator<<(std::ostream& s, const OccupancyGrid& v) {
	Stream(s, "", v);
	return s;
}

/***************************Reflector***************************/
struct Reflector{
	Reflector():id(-1), error(FLT_MAX){}
	Reflector(const int _id,const Point2f& _pt,const float _error)
		:id(_id), pt(_pt), error(_error) {}
	int id;
	Point2f pt;
	float error;
};
static void Stream(std::ostream& s, const std::string& indent, const Reflector& v) {
	s << indent << "id: " << v.id << ",error: " << v.error << ",pt:" << v.pt;
}
inline std::ostream& operator<<(std::ostream& s, const Reflector& v) {
	Stream(s, "", v);
	return s;
}

/***************************ReflectorList***************************/
struct ReflectorList{
	ReflectorList() :mean_error(FLT_MAX) {}
	Header header;
	std::vector<Reflector> reflectors;
	float mean_error;
};
static void Stream(std::ostream& s, const std::string& indent, const ReflectorList& v) {
	s << indent << "header: " << "\n";
	Stream(s, indent + "  ", v.header);
	s << indent << "mean_error: " << v.mean_error << "\n";
	s << indent << "reflectors_size: " << v.reflectors.size() << "\n";
	s << indent << "reflectors: " << "\n";
	for (size_t i = 0; i < v.reflectors.size(); ++i) {
		Stream(s, indent + "  ", v.reflectors[i]);
	}
}
inline std::ostream& operator<<(std::ostream& s, const ReflectorList& v) {
	Stream(s, "", v);
	return s;
}

/***************************Line2D***************************/
struct Line2D{
	robos::Point2d start_point;
	robos::Point2d end_point;
	Line2D() = default;
	Line2D(const robos::Point2d& p1, const robos::Point2d& p2)
		:start_point(p1), end_point(p2) {}
};
static void Stream(std::ostream& s, const std::string& indent, const Line2D& v) {
	s << indent << "start_point: " << v.start_point;
	s << indent << "end_point: " << v.end_point;
}
inline std::ostream& operator<<(std::ostream& s, const Line2D& v) {
	Stream(s, "", v);
	return s;
}
}// namespace robos
#endif // !ROBOS_NAVI_MSGS_H_