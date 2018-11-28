#ifndef CPG_H_
#define CPG_H_
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <vector>

namespace cpg {
    std::vector<std::vector<float>> createK()
    {
        std::vector<std::vector<float>> K;
        K.push_back(std::vector<float>{0, -1, -1, 1, 1, -1});
        K.push_back(std::vector<float>{-1, 0, 1, -1, -1, 1});
        K.push_back(std::vector<float>{-1, 1, 0, -1, -1, 1});
        K.push_back(std::vector<float>{1, -1, -1, 0, 1, -1});
        K.push_back(std::vector<float>{1, -1, -1, 1, 0, -1});
        K.push_back(std::vector<float>{-1, 1, 1, -1, -1, 0});
        return K;
    }

    class CPG {
    public:
        CPG(int legs_number, float w, float gammacpg, float lambdaa, float a, float b, int d,
            float euler_dt, float rk_dt, std::vector<std::vector<float>> K, std::vector<float> cx0,
            std::vector<float> cy0, float kp, float kd, std::vector<float> kpitch, std::vector<float> kroll);
        /**
   * \brief compute the derivative of X and Y, the joints angles. It uses Eq 4 of the paper.
   * \param std::vector<float> X a vector of size legs_number containing the x joints angle (in the
   axial plane)
   * \param std::vector<float> Y a vector of size legs_number containing the y joints angle (in the
   sagittal plane)
   * \param std::vector<float> cx  x center coordinate of cpgg limit cycle
   * \param std::vector<float> cy  y center coordinate of cpgg limit cycle
   * \return std::vector<std::pair<float, float>> XYdot a vector of size legs_number containing the
   pairs(xdot,ydot)
   */
        std::vector<std::pair<float, float>> computeXYdot(std::vector<float> X, std::vector<float> Y);
        std::vector<std::pair<float, float>> computeXYdot(std::vector<float> X, std::vector<float> Y, Eigen::Matrix<float, 1, 6> integrate_delta_thetax, Eigen::Matrix<float, 1, 6> integrate_delta_thetay, Eigen::Matrix<float, 3, 6> delta_theta_e);

        /**
   * \brief Uses Euler integration to obtain x,y from xdot, ydot, xprev, yprev. <br>!!!!! It
   diverges when euler_dt is too small !!!! <br> It would also be better to use Runge Kutta instead
   of Euler
   * \param float xprev : x angle at t-1
   * \param float yprev : y angle at t-1
   * \param std::pair<float, float> xydot : derivative of xprev, yprev obtained with computeXYdot
   * \return std::pair<float, float> return (x,y)
   */
        std::pair<float, float> euler(float xprev, float yprev, std::pair<float, float> xydot);

        /**
   * \brief Uses Runge-Kutta 4 integration to obtain x,y from xdot, ydot, xprev, yprev. <br>!!!!!
   * \param float xprev : x angle at t-1 \param float yprev : y angle at t-1 \param
   * std::pair<float, float> xydot : derivative of xprev, yprev obtained with computeXYdot \param
   * std::pair<float, float> return (x,y)
   */
        std::pair<float, float> RK4(float xprev, float yprev, std::pair<float, float> xydot);

        std::vector<float> computeErrors(float roll, float pitch, std::vector<float> joints);
        std::pair<std::vector<float>, std::vector<float>> computeCPGcmd();

        std::vector<float> get_cx()
        {
            return cx_;
        };

        std::vector<float> get_cy()
        {
            return cy_;
        };

        void set_cy(std::vector<float> cy)
        {
            cy_ = cy;
        };

        void set_rk_dt(float rk_dt)
        {
            rk_dt_ = rk_dt;
        };

    private:
        /*!   number of legs  */
        int legs_number_;
        /*!   angular frequency of the oscillations  */
        float w_;
        /*!   forcing to the limit cycle , a higher value will force to form the ellipse faster. A 0 value
   * is a spiral*/
        float gammacpg_;
        /*!   coupling strength, a higher value will increase the phase shift  */
        float lambda_;
        /*!   major  axes of the limir ellipse */
        float a_;
        /*!   minor axes of the limir ellipse */
        float b_;
        /*!  curvature of the ellipse*/
        int d_;
        /*!  time step in s for euler integration*/
        float euler_dt_;
        /*! time step in s for Runge Kutta 4 integration*/
        float rk_dt_;
        /*!   coupling matrix */
        std::vector<std::vector<float>> K_;
        /*!   center of cpgg limit cycle */
        // std::vector<float> cx0 = {M_PI / 4, -M_PI / 4, 0, 0, -M_PI / 4, M_PI / 4};
        std::vector<float> cx_;
        /*!   center of cpgg limit cycle*/
        std::vector<float> cy_;
        std::vector<float> cx0_;
        /*!   center of cpgg limit cycle*/
        std::vector<float> cy0_;
        // std::vector<float> cy0 = {0, 0, 0, 0, 0, 0};
        float safety_pos_thresh_;
        std::vector<float> Xcommand_;
        std::vector<float> Ycommand_;
        bool integration_has_diverged_;
        std::vector<int> sign_;
        std::vector<float> error_;
        std::vector<float> error_prev_;
        std::vector<float> error_derivated_;
        std::vector<float> error_integrated_;
        std::vector<int> leg_map_to_paper_;
        std::vector<float> kpitch_;
        std::vector<float> kroll_;
        float loop_rate_;
        float kp_;
        float kd_;
    };

    // CPG::CPG(int legs_number = 6, float w = 5, float gammacpg = 7, float lambda = 14, float a = 0.2,
    //     float b = 0.5, int d = 2, float euler_dt = 0.001, float rk_dt = 0.01,
    //     std::vector<std::vector<float>> K = createK(), std::vector<float> cx0 = {0.01, 0.01, 0, 0, 0.01, 0.01},
    //     std::vector<float> cy0 = {-M_PI / 8, -M_PI / 8, -M_PI / 8, -M_PI / 8, -M_PI / 8, -M_PI / 8} // namespace cpg
    //     ) // 10 ou 100

    CPG::CPG(int legs_number = 6, float w = 0.5, float gammacpg = 0.7, float lambda = 0.14, float a = 0.2,
        float b = 0.5, int d = 2, float euler_dt = 0.001, float rk_dt = 0.1,
        std::vector<std::vector<float>> K = createK(), std::vector<float> cx0 = {0.01, 0.0, 0.0, 0.01, 0.01, 0},
        std::vector<float> cy0 = {-M_PI / 8, -M_PI / 8, -M_PI / 8, -M_PI / 8, -M_PI / 8, -M_PI / 8}, float kp = 1, float kd = 0.0,
        std::vector<float> kpitch = {1.4, 0, -1.4, -1.4, 0, 1.4}, std::vector<float> kroll = {1.4, 1.4, 1.4, -1.4, -1.4, -1.4}) // 1

    // CPG::CPG(int legs_number = 6, float w = 3 * 5, float gammacpg = 3 * 7, float lambda = 3 * 14, float a = 0.2,
    //     float b = 0.5, int d = 4, float euler_dt = 0.001, float rk_dt = 0.001,
    //     std::vector<std::vector<float>> K = createK(), std::vector<float> cx0 = {0.1, 0, 0, 0, 0, 0},
    //     std::vector<float> cy0 = {M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10,
    //         M_PI / 10}) 1000
    {
        legs_number_ = legs_number;
        w_ = w;
        gammacpg_ = gammacpg;
        lambda_ = lambda;
        a_ = a;
        b_ = b;
        d_ = d;
        euler_dt_ = euler_dt;
        rk_dt_ = rk_dt;
        K_ = K;
        cx_ = cx0;
        cy_ = cy0;
        cx0_ = cx0;
        cy0_ = cy0;

        kp_ = kp;
        kd_ = kd;

        safety_pos_thresh_ = 1;
        Xcommand_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        Ycommand_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        integration_has_diverged_ = false;
        error_.resize(legs_number * 3, 0.0);
        error_derivated_.resize(legs_number * 3, 0.0);
        error_integrated_.resize(legs_number * 3, 0.0);
        error_prev_.resize(legs_number * 3, 0.0);

        leg_map_to_paper_ = {0, 2, 4, 5, 3, 1};
        kpitch_ = kpitch;
        kroll_ = kroll;
        sign_ = {1, 1, 1, -1, -1, -1};
        loop_rate_ = 0.001;
    }

    /**
 * \brief compute the derivative of X and Y, the joints angles. It uses Eq 4 of the paper.
 * \param std::vector<float> X a vector of size legs_number containing the x joints angle (in the
 axial plane)
 * \param std::vector<float> Y a vector of size legs_number containing the y joints angle (in the
 sagittal plane)
 * \param std::vector<float> cx  x center coordinate of cpgg limit cycle
 * \param std::vector<float> cy  y center coordinate of cpgg limit cycle
 * \return std::vector<std::pair<float, float>> XYdot a vector of size legs_number containing the
 pairs(xdot,ydot)
 */
    std::vector<std::pair<float, float>>
    CPG::computeXYdot(std::vector<float> X, std::vector<float> Y)
    {
        std::vector<std::pair<float, float>> XYdot;
        float x, y, Hcx, dHx, Hcy, dHy, Hc, xdot, ydot, Kterm;

        for (int i = 0; i < K_.size(); i++) {
            x = X[i];
            y = Y[i];
            Hcx = pow((x - cx_[i]) / a_, d_); /* x part of Hc*/
            dHx = d_ * pow((1 / a_), d_) * pow(x, d_ - 1); /* x derivative of Hcx*/

            Hcy = pow((y - cy_[i]) / b_, d_); /* y part of Hc*/
            dHy = d_ * pow((1 / b_), d_) * pow(y, d_ - 1); /* y derivative of Hcx*/

            Hc = Hcx + Hcy;

            xdot = -w_ * dHy + gammacpg_ * (1 - Hc) * dHx;
            // std::cout << xdot << std::endl;
            ydot = w_ * dHx + gammacpg_ * (1 - Hc) * dHy;

            Kterm = 0; /* coupling term which needs to be added to ydot*/
            for (int j = 0; j < K_[i].size(); j++) {
                Kterm += K_[i][j] * (Y[j] - cy_[j]);
                // std::cout << "K_[i][j] " << K_[i][j] << std::endl;
                // std::cout << " K_[i][j] * (Y[j] - cy_[j])] " << K_[i][j] * (Y[j] - cy_[j]) << std::endl;
            }
            // std::cout << " w * dHx    = " << w_ * dHx << '\n';
            // std::cout << "gammacpg_ * (1 - Hc) * dHy   = " << gammacpg_ * (1 - Hc) * dHy << '\n';
            ydot += lambda_ * Kterm;
            // std::cout << "lambda * Kterm    = " << lambda_ * Kterm << '\n';
            // std::cout << " ydot   = " << ydot << '\n';
            XYdot.push_back(std::pair<float, float>(xdot, ydot));
        }
        return XYdot;
    }

    /**
 * \brief compute the derivative of X and Y, the joints angles. It uses Eq 4 of the paper.
 * \param std::vector<float> X a vector of size legs_number containing the x joints angle (in the
 axial plane)
 * \param std::vector<float> Y a vector of size legs_number containing the y joints angle (in the
 sagittal plane)
 * \param std::vector<float> cx  x center coordinate of cpgg limit cycle
 * \param std::vector<float> cy  y center coordinate of cpgg limit cycle
 * \return std::vector<std::pair<float, float>> XYdot a vector of size legs_number containing the
 pairs(xdot,ydot)
 */
    std::vector<std::pair<float, float>>
    CPG::computeXYdot(std::vector<float> X, std::vector<float> Y, Eigen::Matrix<float, 1, 6> integrate_delta_thetax, Eigen::Matrix<float, 1, 6> integrate_delta_thetay, Eigen::Matrix<float, 3, 6> delta_theta_e)
    {
        std::vector<std::pair<float, float>> XYdot;
        float x, y, Hcx, dHx, Hcy, Hcy2x, dHy, Hc, Hc2, xdot, ydot, Kterm;
        int sign = 1;
        float cy2 = 0;
        std::vector<int> leg_map_to_paper = {0, 2, 4, 5, 3, 1};
        for (int i = 0; i < K_.size(); i++) {
            // std::cout << "delta_theta_e " << delta_theta_e(1, i) << std::endl;
            cy_[i] = cy0_[i] + 0.1 * integrate_delta_thetay[i];
            // cx_[i] = cx0_[i] + 0.1 * integrate_delta_thetax[i];
            // cy2 = cy0_[i] + 0.1 * delta_theta_e(1, i);
            // cy2 = cy0_[i] + 0.2 * integrate_delta_theta[i];
            // std::cout << " delta_theta_e " << i << " " << delta_theta_e(1, i) << std::endl;
            sign = (leg_map_to_paper[i] < 3) ? 1 : -1;
            x = X[i];
            y = Y[i];
            Hcx = pow((x - cx_[i]) / a_, d_); /* x part of Hc*/
            dHx = d_ * pow((1 / a_), d_) * pow(x, d_ - 1); /* x derivative of Hcx*/

            Hcy = pow((y - cy_[i]) / b_, d_); /* y part of Hc*/
            // Hcy2x = pow((y - cy2) / b_, d_); /* y part of Hc*/

            dHy = d_ * pow((1 / b_), d_) * pow(y, d_ - 1); /* y derivative of Hcx*/

            Hc = Hcx + Hcy;
            // Hc2 = Hcx + Hcy2x;

            xdot = -w_ * dHy + gammacpg_ * (1 - Hc) * dHx;
            ydot = w_ * dHx + gammacpg_ * (1 - Hc) * dHy;

            Kterm = 0; /* coupling term which needs to be added to ydot*/
            for (int j = 0; j < K_[i].size(); j++) {
                Kterm += K_[i][j] * (Y[j] - cy_[i]);
            }
            std::cout << " w * dHx    = " << w_ * dHx << '\n';
            std::cout << "gammacpg_ * (1 - Hc) * dHy   = " << gammacpg_ * (1 - Hc) * dHy << '\n';
            ydot += lambda_ * Kterm + 0.1 * delta_theta_e(1, i);
            //+0.1 * delta_theta_e(1, i); //_e(1, i);
            std::cout << "delta_theta_e(1, i)= " << i << " " << delta_theta_e(1, i) << '\n';
            // std::cout << "integrate_delta_theta[i]= " << i << " " << integrate_delta_theta[i] << '\n';
            // std::cout << " delta_theta_e(1, i) " << i << " = " << delta_theta_e(1, i) << std::endl;
            std::cout << " xdot " << i << " = " << xdot << '\n';
            std::cout << " ydot   = " << ydot << '\n';
            XYdot.push_back(std::pair<float, float>(xdot, ydot));
        }
        return XYdot;
    }

    /**
 * \brief Uses Euler integation to obtain x,y from xdot, ydot, xprev, yprev. <br>!!!!! It diverges
 when euler_dt is too small !!!! <br> It would also be better to use Runge Kutta instead of Euler
 * \param float xprev : x angle at t-1
 * \param float yprev : y angle at t-1
 * \param std::pair<float, float> xydot : derivative of xprev, yprev obtained with computeXYdot
 * \return std::pair<float, float> return (x,y)
 */
    std::pair<float, float> CPG::euler(float xprev, float yprev, std::pair<float, float> xydot)
    {
        float xcurr = xprev + xydot.first * euler_dt_;
        float ycurr = yprev + xydot.second * euler_dt_;
        return std::pair<float, float>(xcurr, ycurr);
    }

    /**
 * \brief Uses RK4 integation to obtain x,y from xdot, ydot, xprev, yprev. <br>!!!!! It diverges
 when euler_dt is too small !!!! <br> It would also be better to use Runge Kutta instead of Euler
 * \param float xprev : x angle at t-1
 * \param float yprev : y angle at t-1
 * \param std::pair<float, float> xydot : derivative of xprev, yprev obtained with computeXYdot
 * \return std::pair<float, float> return (x,y)
 */
    std::pair<float, float> CPG::RK4(float xprev, float yprev, std::pair<float, float> xydot)
    {
        float k1_x = xydot.first;
        float k2_x = xprev + k1_x * (rk_dt_ / 2);
        float k3_x = xprev + k2_x * (rk_dt_ / 2);
        float k4_x = xprev + k3_x * rk_dt_;
        float xcurr = xprev + (rk_dt_ / 6) * (k1_x + 2 * k2_x + 2 * k3_x + k4_x);

        float k1_y = xydot.second;
        float k2_y = yprev + k1_y * (rk_dt_ / 2);
        float k3_y = yprev + k2_y * (rk_dt_ / 2);
        float k4_y = yprev + k3_y * rk_dt_;
        float ycurr = yprev + (rk_dt_ / 6) * (k1_y + 2 * k2_y + 2 * k3_y + k4_y);

        return std::pair<float, float>(xcurr, ycurr);
    }

    std::pair<std::vector<float>, std::vector<float>>
    CPG::computeCPGcmd()
    {

        for (unsigned int i = 0; i < error_.size(); i++) {
            error_prev_[i] = error_[i];
        }

        /*compute X,Y derivatives*/
        std::vector<std::pair<float, float>> XYdot = CPG::computeXYdot(Xcommand_, Ycommand_);

        for (int i = 0; i < XYdot.size(); i++) {
            /*Integrate XYdot*/
            std::pair<float, float> xy = CPG::RK4(Xcommand_[i], Ycommand_[i], XYdot[i]);
            Xcommand_[i] = xy.first;
            Ycommand_[i] = xy.second;

            if (xy.first > safety_pos_thresh_) {
                Xcommand_[i] = safety_pos_thresh_;
            }
            if (xy.first < -safety_pos_thresh_) {
                Xcommand_[i] = -safety_pos_thresh_;
            }
            if (xy.second > safety_pos_thresh_) {
                Ycommand_[i] = safety_pos_thresh_;
            }
            if (xy.second < -safety_pos_thresh_) {
                Ycommand_[i] = -safety_pos_thresh_;
            }
            if (std::isnan(xy.first) || std::isnan(xy.second)) {
                std::cout << "INTEGRATION HAS DIVERGED : reboot the node and use a bigger loop rate"
                          << std::endl;
                integration_has_diverged_ = true;
            }
        }

        return std::pair<std::vector<float>, std::vector<float>>(Xcommand_, Ycommand_);
    }

    std::vector<float>
    CPG::computeErrors(float roll, float pitch, std::vector<float> joints)
    {
        std::vector<float> command_final;

        for (unsigned int i = 0; i < legs_number_; i++) {
            if (integration_has_diverged_ == false) {
                error_[i] = (joints[i] - sign_[i] * Xcommand_[leg_map_to_paper_[i]]);
                error_derivated_[i] = (error_[i] - error_prev_[i]) / loop_rate_;
                error_integrated_[i] += error_[i];

                command_final.push_back(-kp_ * error_[i] - kd_ * error_derivated_[i]);

                error_[6 + i] = (joints[6 + i] - Ycommand_[leg_map_to_paper_[i]]);
                error_derivated_[6 + i] = (error_[6 + i] - error_prev_[6 + i]) / loop_rate_;
                error_integrated_[6 + i] += error_[6 + i];

                command_final.push_back(-kp_ * error_[6 + i] - kd_ * error_derivated_[6 + i] + kpitch_[i] * pitch + kroll_[i] * roll);

                error_[12 + i] = (joints[12 + i] - Ycommand_[leg_map_to_paper_[i]]);
                error_derivated_[12 + i] = (error_[12 + i] - error_prev_[12 + i]) / loop_rate_;
                error_integrated_[12 + i] += error_[12 + i];

                command_final.push_back(-kp_ * error_[12 + i] - kd_ * error_derivated_[12 + i] + kpitch_[i] * pitch + kroll_[i] * roll);
            }
        }

        return command_final;
    }

} // namespace cpg
#endif
