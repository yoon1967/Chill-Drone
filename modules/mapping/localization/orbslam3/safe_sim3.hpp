#pragma once
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>

namespace ssafy {
// Horn alignment for three RANSAC points. A unit quaternion also represents
// exactly zero rotation: converting through axis / |axis| does not.
inline bool compute_sim3(const Eigen::Matrix3f& p1, const Eigen::Matrix3f& p2,
                         bool fixed_scale, Eigen::Matrix3f& rotation,
                         Eigen::Vector3f& translation, float& scale,
                         Eigen::Matrix4f& forward, Eigen::Matrix4f& inverse) {
    if (!p1.allFinite() || !p2.allFinite()) return false;
    const Eigen::Vector3d c1 = p1.cast<double>().rowwise().mean();
    const Eigen::Vector3d c2 = p2.cast<double>().rowwise().mean();
    const Eigen::Matrix3d a = p1.cast<double>().colwise() - c1;
    const Eigen::Matrix3d b = p2.cast<double>().colwise() - c2;
    for (const Eigen::Matrix3d* points : {&a, &b}) {
        const double spread = points->squaredNorm();
        if (!(spread > 0) || !std::isfinite(spread) ||
            points->col(0).cross(points->col(1)).squaredNorm() <= 1e-10 * spread * spread)
            return false;
    }
    const Eigen::Matrix3d m = b * a.transpose();
    Eigen::Matrix4d n;
    n << m.trace(), m(1,2)-m(2,1), m(2,0)-m(0,2), m(0,1)-m(1,0),
         m(1,2)-m(2,1), m(0,0)-m(1,1)-m(2,2), m(0,1)+m(1,0), m(2,0)+m(0,2),
         m(2,0)-m(0,2), m(0,1)+m(1,0), -m(0,0)+m(1,1)-m(2,2), m(1,2)+m(2,1),
         m(0,1)-m(1,0), m(2,0)+m(0,2), m(1,2)+m(2,1), -m(0,0)-m(1,1)+m(2,2);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(n);
    if (solver.info() != Eigen::Success || !solver.eigenvectors().allFinite()) return false;
    const Eigen::Vector4d q = solver.eigenvectors().col(3);
    const Eigen::Matrix3d r = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized().toRotationMatrix();
    const Eigen::Matrix3d rotated = r * b;
    const double s = fixed_scale ? 1. : (a.array() * rotated.array()).sum() / rotated.squaredNorm();
    if (!std::isfinite(s) || s <= 0) return false;
    const Eigen::Vector3d t = c1 - s * r * c2;
    rotation = r.cast<float>(); translation = t.cast<float>(); scale = float(s);
    forward.setIdentity(); inverse.setIdentity();
    forward.block<3,3>(0,0) = scale * rotation;
    forward.block<3,1>(0,3) = translation;
    inverse.block<3,3>(0,0) = rotation.transpose() / scale;
    inverse.block<3,1>(0,3) = -inverse.block<3,3>(0,0) * translation;
    return forward.allFinite() && inverse.allFinite();
}
}
