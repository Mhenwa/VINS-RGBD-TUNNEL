#pragma once

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>

struct DepthToMapFactor
{
    DepthToMapFactor(const Eigen::Vector3d &point_c,
                     const Eigen::Vector4d &plane,
                     double scale,
                     double sqrt_info)
        : point_c_(point_c), plane_(plane), scale_(scale), sqrt_info_(sqrt_info)
    {
    }

    template <typename T>
    bool operator()(const T *const pose, const T *const ex_pose, T *residuals) const
    {
        Eigen::Matrix<T, 3, 1> P(pose[0], pose[1], pose[2]);
        Eigen::Quaternion<T> Q(pose[6], pose[3], pose[4], pose[5]);
        Eigen::Matrix<T, 3, 1> tic(ex_pose[0], ex_pose[1], ex_pose[2]);
        Eigen::Quaternion<T> qic(ex_pose[6], ex_pose[3], ex_pose[4], ex_pose[5]);

        Eigen::Matrix<T, 3, 1> point_c(T(point_c_(0)), T(point_c_(1)), T(point_c_(2)));
        Eigen::Matrix<T, 3, 1> point_w = Q * (qic * point_c + tic) + P;

        residuals[0] = T(sqrt_info_ * scale_) *
                       (T(plane_(0)) * point_w.x() +
                        T(plane_(1)) * point_w.y() +
                        T(plane_(2)) * point_w.z() +
                        T(plane_(3)));
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d &point_c,
                                       const Eigen::Vector4d &plane,
                                       double scale,
                                       double sqrt_info)
    {
        return new ceres::AutoDiffCostFunction<DepthToMapFactor, 1, 7, 7>(
            new DepthToMapFactor(point_c, plane, scale, sqrt_info));
    }

    Eigen::Vector3d point_c_;
    Eigen::Vector4d plane_;
    double scale_;
    double sqrt_info_;
};
