#include "kinematics.hpp"
#include <cassert>
#include <cmath>
int main(){
  auto I=kin::Mat4::identity();assert(std::fabs(I.m[0][0]-1)<1e-12);
  auto A=kin::dhMatrix(0,0.1,0.2,0);assert(std::fabs(A.m[0][3]-0.2)<1e-12);assert(std::fabs(A.m[2][3]-0.1)<1e-12);
  kin::JointAngles q{0,0,0,0};auto T=kin::forward(q);auto p=T.position();assert(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z));
  std::string why;assert(!kin::checkPath({{0,0,0}},kin::kToolDown,0,&why));
}