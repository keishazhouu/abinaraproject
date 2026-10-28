#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <array>
#include <string>
#include <vector>

namespace kin {
constexpr double kPi = 3.14159265358979323846;
constexpr double kToolDown = -kPi / 2.0;

struct Vec3 { double x=0.0, y=0.0, z=0.0; };
Vec3 operator+(const Vec3&, const Vec3&);
Vec3 operator-(const Vec3&, const Vec3&);
Vec3 operator*(const Vec3&, double);
double norm(const Vec3&);
double distance(const Vec3&, const Vec3&);

struct Mat4 {
  double m[4][4]{};
  static Mat4 identity();
  Mat4 operator*(const Mat4& o) const;
  Vec3 position() const { return {m[0][3],m[1][3],m[2][3]}; }
  Vec3 axisX() const { return {m[0][0],m[1][0],m[2][0]}; }
};
using JointAngles = std::array<double,4>;
struct DHRow { const char* name; double theta_offset, d, a, alpha; };
struct JointLimit { double lo, hi; };
const std::array<DHRow,4>& dhTable();
const std::array<JointLimit,4>& jointLimits();
Mat4 dhMatrix(double theta, double d, double a, double alpha);
Mat4 forward(const JointAngles& q);
std::array<Mat4,4> forwardFrames(const JointAngles& q);
double toolPitch(const Mat4& T);
struct IKResult { bool ok=false; JointAngles q{}; std::string message; };
IKResult inverse(const Vec3& p, double tool_pitch, const JointAngles* previous=nullptr);
bool checkPath(const std::vector<Vec3>& points, double tool_pitch, double step, std::string* why=nullptr);
double radToDeg(double r);
double degToRad(double d);
}
#endif