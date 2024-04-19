#ifndef CERESFACTORS_H
#define CERESFACTORS_H
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "sophus/se3.hpp"
#include "Thirdparty/Sophus/sophus/geometry.hpp"

namespace ORB_SLAM2
{

Eigen::Matrix3d Skew(const Eigen::Vector3d &w);


class MonoOnlyPoseFactor : public ceres::SizedCostFunction<2,7>
{
  public:
    MonoOnlyPoseFactor(const Eigen::Vector3d &Pw_, const Eigen::Vector2d &obs_, const Eigen::Matrix2d &sqrt_info_, const double &fx_, const double &fy_,const double &cx_,const double &cy_);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;


    Eigen::Vector3d Pw;
    Eigen::Matrix2d sqrt_info;
    Eigen::Vector2d obs;
    double fx;
    double fy;
    double cx;
    double cy;
};

class StereoOnlyPoseFactor : public ceres::SizedCostFunction<2,7>
{
  public:
    StereoOnlyPoseFactor(const Eigen::Vector3d &Pw_, const Eigen::Vector3d &obs_, const Eigen::Matrix3d &sqrt_info_, const double &b_, const double &fx_, const double &fy_,const double &cx_,const double &cy_);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;


    Eigen::Vector3d Pw;
    Eigen::Matrix3d sqrt_info;
    Eigen::Vector3d obs;
    double fx;
    double fy;
    double cx;
    double cy;
    double b;
};


}


#endif