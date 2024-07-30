#ifndef ROBOS_SRV_MSGS_H_
#define ROBOS_SRV_MSGS_H_
#include "robos/geometry_msgs.h"
namespace robos {
/***************************RelocalizationData***************************/
struct RelocalizationData2D {
	RelocalizationData2D()
		:linear_search_window(0.f)
		, angular_search_window(0.f) {}
	Pose2D initial_pose;
	float linear_search_window;
	float angular_search_window;
};
inline bool operator==(const RelocalizationData2D& lhs, const RelocalizationData2D& rhs) {
	return lhs.initial_pose == rhs.initial_pose
		&& lhs.linear_search_window == rhs.linear_search_window
		&& lhs.angular_search_window == rhs.angular_search_window;
}
inline bool operator!=(const RelocalizationData2D& lhs, const RelocalizationData2D& rhs) {
	return !(lhs == rhs);
}
static void Stream(std::ostream& s, const std::string& indent, const RelocalizationData2D& v) {
	s << indent << "initial_pose: " << v.initial_pose;
	s << indent << "linear_search_window: " << v.linear_search_window << "\n";
	s << indent << "angular_search_window: " << v.angular_search_window << "\n";
}
inline std::ostream& operator<<(std::ostream& s, const RelocalizationData2D& v) {
	Stream(s, "", v);
	return s;
}
}  // namespace robos
#endif  // ROBOS_SRV_MSGS_H_
