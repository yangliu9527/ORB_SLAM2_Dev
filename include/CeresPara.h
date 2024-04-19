#ifndef CERESPARA_H
#define CERESPARA_H
#include <ceres/ceres.h>
#include <Eigen/Dense>

namespace ORB_SLAM2
{

    // Sophus::SE3中的SO3为Eigen::Quaterniond, 初始化顺序为Eigen::Quaterniond(qw,qx,qy,qz), 但内部存储顺序为.data() = (qx,qy,qz,qw)
    // Sophus::SE3::data()返回的数组顺序为(x,y,z,qw,qx,qy,qz)
    // Sophus::SE3::exp(x)中的x顺序为(rho, phi), rho对应平移, phi对应旋转
    // 规定待优化参数parameters与Sophus::SE3::data()一致 [x,y,z,qw,qx,qy,qz]
    // 规定李代数为先平移再旋转，与Sophus::SE3::exp(x)一致 [rho, phi]

    class SE3Parameterization : public ceres::Manifold
    {
    public:
        virtual ~SE3Parameterization() {}

        virtual bool Plus(const double *x,
                          const double *delta,
                          double *x_plus_delta) const override
        {
            const Eigen::Map<const Sophus::SE3d> T(x);
            Eigen::Map<Sophus::SE3d> T_plus_delta(x_plus_delta);
            const Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta_T(delta);
            T_plus_delta = Sophus::SE3d::exp(delta_T) * T; // left multiple
            return true;
        }

        // 外部残差直接对李代数求雅可比，所以这里设为单位阵即可
        virtual bool PlusJacobian(const double *x, double *jacobian) const override
        {
            const Eigen::Map<const Sophus::SE3d> T(x);
            Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(jacobian);
            J.setZero();
            J.block<3, 3>(0, 0).setIdentity();
            J.block<3, 3>(4, 3).setIdentity();
            // jaco_tmp = T.Dx_this_mul_exp_x_at_0();
            return true;
        }

        virtual int AmbientSize() const override { return Sophus::SE3d::num_parameters; }

        virtual int TangentSize() const override { return Sophus::SE3d::DoF; }

        virtual bool RightMultiplyByPlusJacobian(const double *x,
                                                 const int num_rows,
                                                 const double *ambient_matrix,
                                                 double *tangent_matrix) const override { return true; }
        virtual bool Minus(const double *y, const double *x, double *y_minus_x) const override { return true; }
        virtual bool MinusJacobian(const double *x, double *jacobian) const override { return true; }
    };

}

#endif