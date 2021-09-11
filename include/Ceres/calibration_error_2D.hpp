#ifndef CALIBRATION_ERROR_2D_HPP
#define CALIBRATION_ERROR_2D_HPP

#include "Eigen/Core"
#include "ceres/ceres.h"
namespace ceres
{
    template <typename T>
    Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw_radians)
    {
        const T cos_yaw = ceres::cos(yaw_radians);
        const T sin_yaw = ceres::sin(yaw_radians);

        Eigen::Matrix<T, 2, 2> rotation;
        rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
        return rotation;
    }
    class Calibration2DErrorTerm
    {
    private:
        double x_u_, y_u_, x_c_, y_c_;

    public:
        Calibration2DErrorTerm(double x_u, double y_u, double x_c, double y_c)
            : x_u_(x_u), y_u_(y_u), x_c_(x_c), y_c_(y_c) {}

        template <typename T>
        bool operator()(const T *const yaw, const T *const t, T *residuals_ptr) const
        {
            Eigen::Matrix<T, 2, 1> translation{t[0], t[1]};
            Eigen::Matrix<T, 2, 2> roatation;
            Eigen::Matrix<T, 2, 1> p_u{T(x_u_), T(y_u_)};
            Eigen::Matrix<T, 2, 1> p_c{T(x_c_), T(y_c_)};
            Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residuals_ptr);

            residuals_map = RotationMatrix2D(*yaw) * p_u + translation - p_c;
            return true;
        }

        static ceres::CostFunction *Create(double x_u, double y_u, double x_c, double y_c)
        {
            return (new ceres::AutoDiffCostFunction<Calibration2DErrorTerm, 2, 1, 2>
            (new Calibration2DErrorTerm(x_u, y_u, x_c, y_c)));
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace ceres
#endif