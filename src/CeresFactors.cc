#include "CeresFactors.h"

namespace ORB_SLAM2
{
    Eigen::Matrix3d Skew(const Eigen::Vector3d &w)
    {
        Eigen::Matrix3d W;
        W << 0.0, -w[2], w[1], w[2], 0.0, -w[0], -w[1], w[0], 0.0;
        return W;
    }

    // Sophus::SE3中的SO3为Eigen::Quaterniond, 初始化顺序为Eigen::Quaterniond(qw,qx,qy,qz), 但内部存储顺序为.data() = (qx,qy,qz,qw)
    // Sophus::SE3::data()返回的数组顺序为(x,y,z,qw,qx,qy,qz)
    // Sophus::SE3::exp(x)中的x顺序为(rho, phi), rho对应平移, phi对应旋转
    // 规定待优化参数parameters与Sophus::SE3::data()一致 [x,y,z,qw,qx,qy,qz]
    // 规定李代数为先平移再旋转，与Sophus::SE3::exp(x)一致 [rho, phi]

    MonoOnlyPoseFactor::MonoOnlyPoseFactor(const Eigen::Vector3d &Pw_, const Eigen::Vector2d &obs_, const Eigen::Matrix2d &sqrt_info_, const double &fx_, const double &fy_, const double &cx_, const double &cy_) : Pw(Pw_), obs(obs_), sqrt_info(sqrt_info_), fx(fx_), fy(fy_), cx(cx_), cy(cy_)
    {
    }

    bool MonoOnlyPoseFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        const Eigen::Map<const Sophus::SE3d> Tcw(parameters[0]);
        Eigen::Vector3d Pc = Tcw * Pw;
        Eigen::Vector2d Pc_norm = (Pc / Pc[2]).block<2, 1>(0, 0);
        Eigen::Vector2d uv(fx * Pc_norm[0] + cx, fy * Pc_norm[1] + cy);
        Eigen::Map<Eigen::Vector2d> residual(residuals);
        residual = uv - obs;
        residual = sqrt_info*residual;

        if (jacobians)
        {
            double inv_Z = 1 / Pc[2];
            double inv_Z2 = 1 / (Pc[2] * Pc[2]);
            Eigen::Matrix<double, 2, 3> proj_jac;
            proj_jac << fx * inv_Z, 0, -fx * Pc[0] * inv_Z2,
                0, fy * inv_Z, -fy * Pc[1] * inv_Z2;
            Eigen::Matrix<double, 3, 7> dPc_dTcw;
            dPc_dTcw.block<3,3>(0,0).setIdentity();
            dPc_dTcw.block<3,3>(0,4) = -Skew(Pc);
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> Jac(jacobians[0]);
            Jac = proj_jac*dPc_dTcw;
            
        }

        return true;
    }

    StereoOnlyPoseFactor::StereoOnlyPoseFactor(const Eigen::Vector3d &Pw_, const Eigen::Vector3d &obs_, const Eigen::Matrix3d &sqrt_info_, const double &b_, const double &fx_, const double &fy_, const double &cx_, const double &cy_) : Pw(Pw_), obs(obs_), sqrt_info(sqrt_info_), b(b_), fx(fx_), fy(fy_), cx(cx_), cy(cy_)
    {
    }

    bool StereoOnlyPoseFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        const Eigen::Map<const Sophus::SE3d> Tcw(parameters[0]);
        Eigen::Vector3d Pc = Tcw * Pw;
        Eigen::Vector2d Pc_norm = (Pc / Pc[2]).block<2, 1>(0, 0);
        Eigen::Vector3d uv(fx * Pc_norm[0] + cx, fy * Pc_norm[1] + cy, fx*(Pc[0]-b)/Pc[2]+cx);
        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual = uv - obs;
        residual = sqrt_info*residual;

        if (jacobians)
        {
            double inv_Z = 1 / Pc[2];
            double inv_Z2 = 1 / (Pc[2] * Pc[2]);
            Eigen::Matrix<double, 3, 3> proj_jac;
            proj_jac << fx * inv_Z, 0, -fx * Pc[0] * inv_Z2,
                0, fy * inv_Z, -fy * Pc[1] * inv_Z2,
                fx*inv_Z, 0, -fx*(Pc[0]-b)*inv_Z2;
            Eigen::Matrix<double, 3, 7> dPc_dTcw;
            dPc_dTcw.block<3,3>(0,0).setIdentity();
            dPc_dTcw.block<3,3>(0,4) = -Skew(Pc);
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> Jac(jacobians[0]);
            Jac = proj_jac*dPc_dTcw; 
        }

        return true;
    }

}