#include "kinematics.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace kin {
namespace {
constexpr double kTorsoZ=0.1, kShoulderOnTorso=0.375;
constexpr double kElbowX=-0.0175, kElbowZ=0.275, kForearm=0.2, kTool=0.08;
constexpr double kUpperQuatAngle=1.0, kLowerQuatAngle=1.15;
double upperLength(){ return std::hypot(kElbowX,kElbowZ); }
double elbowSkew(){ return std::atan2(kElbowX,kElbowZ); }
double wrapAngle(double a){ a=std::fmod(a+kPi,2*kPi); if(a<0) a+=2*kPi; return a-kPi; }
}
Vec3 operator+(const Vec3&a,const Vec3&b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
Vec3 operator-(const Vec3&a,const Vec3&b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
Vec3 operator*(const Vec3&a,double s){return {a.x*s,a.y*s,a.z*s};}
double norm(const Vec3&a){return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);}
double distance(const Vec3&a,const Vec3&b){return norm(a-b);}
Mat4 Mat4::identity(){Mat4 r{}; for(int i=0;i<4;++i) r.m[i][i]=1; return r;}
Mat4 Mat4::operator*(const Mat4&o)const{Mat4 r{};for(int i=0;i<4;++i)for(int j=0;j<4;++j)for(int k=0;k<4;++k)r.m[i][j]+=m[i][k]*o.m[k][j];return r;}
const std::array<DHRow,4>& dhTable(){
 static const std::array<DHRow,4> t={{{"hip",0.0,kTorsoZ+kShoulderOnTorso,0.0,-kPi/2},
 {"shoulder",kUpperQuatAngle+elbowSkew()-kPi/2,0.0,upperLength(),0.0},
 {"elbow",kLowerQuatAngle-elbowSkew(),0.0,kForearm,0.0},
 {"wrist",0.0,0.0,kTool,0.0}}}; return t;
}
const std::array<JointLimit,4>& jointLimits(){static const std::array<JointLimit,4> l={{{-1e9,1e9},{-1.5708,0.6981},{-2.25,1.0},{-1.5708,1.5708}}};return l;}
Mat4 dhMatrix(double th,double d,double a,double al){double ct=cos(th),st=sin(th),ca=cos(al),sa=sin(al);Mat4 T=Mat4::identity();T.m[0][0]=ct;T.m[0][1]=-st*ca;T.m[0][2]=st*sa;T.m[0][3]=a*ct;T.m[1][0]=st;T.m[1][1]=ct*ca;T.m[1][2]=-ct*sa;T.m[1][3]=a*st;T.m[2][0]=0;T.m[2][1]=sa;T.m[2][2]=ca;T.m[2][3]=d;return T;}
std::array<Mat4,4> forwardFrames(const JointAngles&q){const auto&t=dhTable();std::array<Mat4,4> f{};Mat4 T=Mat4::identity();for(size_t i=0;i<4;++i){T=T*dhMatrix(q[i]+t[i].theta_offset,t[i].d,t[i].a,t[i].alpha);f[i]=T;}return f;}
Mat4 forward(const JointAngles&q){return forwardFrames(q)[3];}
double toolPitch(const Mat4&T){Vec3 x=T.axisX();return std::atan2(x.z,std::hypot(x.x,x.y));}

IKResult inverse(const Vec3&p,double pitch,const JointAngles*previous){
 const auto&t=dhTable();const auto&lim=jointLimits();IKResult out;
 const double r=std::hypot(p.x,p.y);double q1=(r>1e-9)?std::atan2(p.y,p.x):(previous?(*previous)[0]:0.0);
 if(previous) q1+=2*kPi*std::round(((*previous)[0]-q1)/(2*kPi));
 const double a2=t[1].a,a3=t[2].a,a4=t[3].a,d1=t[0].d;
 const double wx=r-a4*std::cos(pitch), wy=-(p.z-d1)-a4*std::sin(-pitch);
 const double c=(wx*wx+wy*wy-a2*a2-a3*a3)/(2*a2*a3);
 if(!std::isfinite(c)||std::fabs(c)>1.0){out.message="Target di luar jangkauan geometri IK.";out.q={q1,0,0,0};return out;}
 struct Candidate{JointAngles q;int bad;};std::array<Candidate,2> candidates{};int n=0;
 for(double sign:{1.0,-1.0}){
   const double th3=sign*std::acos(std::clamp(c,-1.0,1.0));
   const double th2=std::atan2(wy,wx)-std::atan2(a3*std::sin(th3),a2+a3*std::cos(th3));
   JointAngles q{q1,wrapAngle(th2-t[1].theta_offset),wrapAngle(th3-t[2].theta_offset),wrapAngle(-pitch-th2-th3-t[3].theta_offset)};
   int bad=-1;for(int j=1;j<4;++j)if(q[j]<lim[j].lo||q[j]>lim[j].hi){bad=j;break;}
   candidates[n++]={q,bad};
 }
 bool ok0=candidates[0].bad<0,ok1=candidates[1].bad<0;
 if(!ok0&&!ok1){out.q=candidates[0].q;char b[180];int j=candidates[0].bad;std::snprintf(b,sizeof(b),"Kandidat IK melanggar limit joint %s (%.2f deg).",t[j].name,radToDeg(out.q[j]));out.message=b;return out;}
 int pick=ok0?0:1;if(ok0&&ok1&&previous){double s[2]={0,0};for(int j=0;j<4;++j){s[0]+=std::fabs(candidates[0].q[j]-(*previous)[j]);s[1]+=std::fabs(candidates[1].q[j]-(*previous)[j]);}pick=s[1]<s[0]?1:0;}
 out.ok=true;out.q=candidates[pick].q;out.message="OK";return out;
}
bool checkPath(const std::vector<Vec3>&pts,double pitch,double step,std::string*why){if(!(step>0)||!std::isfinite(step)){if(why)*why="step harus bernilai positif dan finite.";return false;}if(pts.empty()){if(why)*why="Trajectory kosong.";return false;}for(size_t i=0;i<pts.size();++i){const Vec3&a=pts[i];const Vec3&b=(i+1<pts.size())?pts[i+1]:pts[i];int n=std::max(1,static_cast<int>(std::ceil(distance(a,b)/step)));for(int k=0;k<=n;++k){Vec3 p=a+(b-a)*(double(k)/n);auto r=inverse(p,pitch);if(!r.ok){if(why)*why=r.message;return false;}}}return true;}
double radToDeg(double r){return r*180/kPi;}double degToRad(double d){return d*kPi/180;}
}
