#include "safe_sim3.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

static void check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
int main() {
    Eigen::Matrix3f p, r; Eigen::Vector3f t; float s; Eigen::Matrix4f f, inv;
    p << 0, 1, .2, 0, .2, 1, 1, 1.5, 2;
    // Reproduce the old zero-angle 0/0 path with exact identity and translation.
    check(ssafy::compute_sim3(p,p,false,r,t,s,f,inv), "identity rejected");
    check(f.isApprox(Eigen::Matrix4f::Identity(), 1e-5f), "identity incorrect");
    Eigen::Vector3f shift(1, -2, 3);
    Eigen::Matrix3f shifted = p.colwise() + shift;
    check(ssafy::compute_sim3(shifted,p,true,r,t,s,f,inv), "translation rejected");
    check(t.isApprox(shift,1e-5f) && r.isApprox(Eigen::Matrix3f::Identity(),1e-5f), "translation incorrect");
    for(float angle : {0.f, 1e-7f, .5f, 3.1415926f}) {
        Eigen::Matrix3f expected = Eigen::AngleAxisf(angle, Eigen::Vector3f(1,2,3).normalized()).toRotationMatrix();
        Eigen::Matrix3f target = (2.5f * expected * p).colwise() + shift;
        check(ssafy::compute_sim3(target,p,false,r,t,s,f,inv), "rotation/scale rejected");
        check(r.isApprox(expected,1e-4f) && std::abs(s-2.5f)<1e-4f && t.isApprox(shift,1e-4f), "Sim3 incorrect");
        check((f*inv).isApprox(Eigen::Matrix4f::Identity(),1e-4f), "inverse incorrect");
    }
    Eigen::Matrix3f bad = Eigen::Matrix3f::Zero();
    check(!ssafy::compute_sim3(bad,p,false,r,t,s,f,inv), "coincident accepted");
    bad << 0,1,2, 0,1,2, 0,1,2;
    check(!ssafy::compute_sim3(bad,p,false,r,t,s,f,inv), "collinear accepted");
    bad=p; bad(0,0)=std::numeric_limits<float>::quiet_NaN();
    check(!ssafy::compute_sim3(bad,p,false,r,t,s,f,inv), "NaN accepted");
    bad=p; bad(0,0)=std::numeric_limits<float>::infinity();
    check(!ssafy::compute_sim3(bad,p,false,r,t,s,f,inv), "infinity accepted");
    std::cout << "Sim3 identity, translation, rotation, scale and degeneracy checks passed\n";
}
