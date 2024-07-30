#ifndef ROBOS_STD_MSGS_H_
#define ROBOS_STD_MSGS_H_
#include <string>
#include "robos/time.h"
namespace robos{
/***************************Header***************************/
struct Header{
	Time stamp;
	std::string frame_id;
};
inline bool operator==(const Header& lhs, const Header& rhs){
	return lhs.stamp == rhs.stamp && lhs.frame_id == rhs.frame_id;
}
inline bool operator!=(const Header& lhs, const Header& rhs){
	return !(lhs == rhs);
}
static void Stream(std::ostream& s, const std::string& indent, const Header& v) {
	s << indent << "stamp: " << v.stamp << "\n";
	s << indent << "frame_id: " << v.frame_id << "\n";
}
inline std::ostream& operator<<(std::ostream& s, const Header& v) {
	Stream(s, "", v);
	return s;
}
} // namespace robos
#endif // !ROBOS_STD_MSGS_H_