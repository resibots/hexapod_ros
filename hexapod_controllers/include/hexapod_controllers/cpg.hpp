#ifndef CPG_H_
#define CPG_H_
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <vector>

namespace cpg {
std::vector<std::vector<float>> createK() {
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
  CPG() = delete;

  CPG(int legs_number, float w, float gammacpg, float lambdaa, float a, float b, int d,
      float euler_dt, float rk_dt, std::vector<std::vector<float>> K, std::vector<float> cx0,
      std::vector<float> cy0, std::vector<float> X0, std::vector<float> Y0);

  /**
   * \brief initialize K with default values from the paper
   * \param std::vector<std::vector<float>> where a vector of lines
   * \return void, K is passed by reference
   */
  void initK(std::vector<std::vector<float>> &K);

  /**
   * \brief compute the derivative of X and Y, the joints angles. It uses Eq 4 of the paper.
   * \param std::vector<float> X a vector of size legs_number containing the x joints angle (in the
   axial plane)
   * \param std::vector<float> Y a vector of size legs_number containing the y joints angle (in the
   sagittal plane)
   * \param float w angular frequency of the oscillations
   * \param float gammacpg forcing to the limit cycle
   * \param float lambda coupling strength
   * \param float a major  axes of the limit ellipse
   * \param float b minor  axes of the limit ellipse
   * \param int d curvature of the ellipse
   * \param std::vector<std::vector<float>> K coupling matrix
   * \param std::vector<float> cx  x center coordinate of cpgg limit cycle
   * \param std::vector<float> cy  y center coordinate of cpgg limit cycle
   * \return std::vector<std::pair<float, float>> XYdot a vector of size legs_number containing the
   pairs(xdot,ydot)
   */
  std::vector<std::pair<float, float>> computeXYdot(std::vector<float> X, std::vector<float> Y,
                                                    float w, float gammacpg, float lambda, float a,
                                                    float b, int d,
                                                    std::vector<std::vector<float>> K,
                                                    std::vector<float> cx, std::vector<float> cy);

  /**
   * \brief Uses Euler integration to obtain x,y from xdot, ydot, xprev, yprev. <br>!!!!! It
   diverges when euler_dt is too small !!!! <br> It would also be better to use Runge Kutta instead
   of Euler
   * \param float xprev : x angle at t-1
   * \param float yprev : y angle at t-1
   * \param std::pair<float, float> xydot : derivative of xprev, yprev obtained with computeXYdot
   * \param float euler_dt time step for euler integration : it is better to take less than 0.01
   * \return std::pair<float, float> return (x,y)
   */
  std::pair<float, float> euler(float xprev, float yprev, std::pair<float, float> xydot,
                                float euler_dt);

  /**
   * \brief Uses Runge-Kutta 4 integration to obtain x,y from xdot, ydot, xprev, yprev. <br>!!!!!
   * \param float xprev : x angle at t-1 \param float yprev : y angle at t-1 \param
   * std::pair<float, float> xydot : derivative of xprev, yprev obtained with computeXYdot \param
   * float rk_dt time step for RK4 integration : it is better to take less than 0.01 \return
   * std::pair<float, float> return (x,y)
   */
  std::pair<float, float> RK4(float xprev, float yprev, std::pair<float, float> xydot, float rk_dt);

private:
  /*!   number of legs  */
  int legs_number_;
  /*!   angular frequency of the oscillations  */
  float w_;
  /*!   forcing to the limit cycle , a higher value will force to form the ellipse faster. A 0 value
   * is a spiral*/
  float gammacpg_;
  /*!   coupling strength, a higher value will increase the phase shift  */
  float lambd_;
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
  std::vector<float> cx0_;
  /*!   center of cpgg limit cycle*/
  std::vector<float> cy0_;
  // std::vector<float> cy0 = {0, 0, 0, 0, 0, 0};
  /*!   angles of the soulder joints in the axial plane at init*/
  std::vector<float> X0_;
  /*!   angles of the soulder joints in the sagittal plane at init*/
  std::vector<float> Y0_;
};

CPG::CPG(int legs_number = 6, float w = 5, float gammacpg = 40, float lambda = 0.25, float a = 0.2,
         float b = 0.5, int d = 2, float euler_dt = 0.0001, float rk_dt = 0.0001,
         std::vector<std::vector<float>> K = createK(), std::vector<float> cx0 = {0, 0, 0, 0, 0, 0},
         std::vector<float> cy0 = {M_PI / 16, M_PI / 16, M_PI / 16, M_PI / 16, M_PI / 16,
                                   M_PI / 16},
         std::vector<float> X0 = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
         std::vector<float> Y0 = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1}) {
  legs_number_ = legs_number;
  w_ = w;
  gammacpg_ = gammacpg;
  lambd_ = lambda;
  a_ = a;
  b_ = b;
  d_ = d;
  euler_dt_ = euler_dt;
  rk_dt_ = rk_dt;
  K_ = K;
  cx0_ = cx0;
  cy0_ = cy0;
  X0_ = X0;
  Y0_ = Y0;
}

/**
 * \brief initialize K with default values from the paper
 * \param std::vector<std::vector<float>> where a vector of lines
 * \return void, K is passed by reference
 */
void CPG::initK(std::vector<std::vector<float>> &K) {
  K.push_back(std::vector<float>{0, -1, -1, 1, 1, -1});
  K.push_back(std::vector<float>{-1, 0, 1, -1, -1, 1});
  K.push_back(std::vector<float>{-1, 1, 0, -1, -1, 1});
  K.push_back(std::vector<float>{1, -1, -1, 0, 1, -1});
  K.push_back(std::vector<float>{1, -1, -1, 1, 0, -1});
  K.push_back(std::vector<float>{-1, 1, 1, -1, -1, 0});
}

/**
 * \brief compute the derivative of X and Y, the joints angles. It uses Eq 4 of the paper.
 * \param std::vector<float> X a vector of size legs_number containing the x joints angle (in the
 axial plane)
 * \param std::vector<float> Y a vector of size legs_number containing the y joints angle (in the
 sagittal plane)
 * \param float w angular frequency of the oscillations
 * \param float gammacpg forcing to the limit cycle
 * \param float lambda coupling strength
 * \param float a major  axes of the limit ellipse
 * \param float b minor  axes of the limit ellipse
 * \param int d curvature of the ellipse
 * \param std::vector<std::vector<float>> K coupling matrix
 * \param std::vector<float> cx  x center coordinate of cpgg limit cycle
 * \param std::vector<float> cy  y center coordinate of cpgg limit cycle
 * \return std::vector<std::pair<float, float>> XYdot a vector of size legs_number containing the
 pairs(xdot,ydot)
 */
std::vector<std::pair<float, float>>
CPG::computeXYdot(std::vector<float> X, std::vector<float> Y, float w, float gammacpg, float lambda,
                  float a, float b, int d, std::vector<std::vector<float>> K, std::vector<float> cx,
                  std::vector<float> cy) {
  std::vector<std::pair<float, float>> XYdot;
  float x, y, Hcx, dHx, Hcy, dHy, Hc, xdot, ydot, Kterm;

  for (int i = 0; i < K.size(); i++) {
    x = X[i];
    y = Y[i];
    Hcx = pow((x - cx[i]) / a, d);             /* x part of Hc*/
    dHx = d * pow((1 / a), d) * pow(x, d - 1); /* x derivative of Hcx*/

    Hcy = pow((y - cy[i]) / b, d);             /* y part of Hc*/
    dHy = d * pow((1 / b), d) * pow(y, d - 1); /* y derivative of Hcx*/

    Hc = Hcx + Hcy;

    xdot = -w * dHy + gammacpg * (1 - Hc) * dHx;

    ydot = w * dHx + gammacpg * (1 - Hc) * dHy;

    Kterm = 0; /* coupling term which needs to be added to ydot*/
    for (int j = 0; j < K[i].size(); j++) {
      Kterm += K[i][j] * (Y[j] - cy[j]);
    }
    // std::cout << " w * dHx    = " << w * dHx << '\n';
    // std::cout << "gammacpg * (1 - Hc) * dHy   = " << gammacpg * (1 - Hc) * dHy << '\n';
    ydot += lambda * Kterm;
    // std::cout << "lambda * Kterm    = " << lambda * Kterm << '\n';
    // std::cout << " ydot   = " << ydot << '\n';
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
 * \param float euler_dt time step for euler integration : it is better to take less than 0.01
 * \return std::pair<float, float> return (x,y)
 */
std::pair<float, float> CPG::euler(float xprev, float yprev, std::pair<float, float> xydot,
                                   float euler_dt) {
  float xcurr = xprev + xydot.first * euler_dt;
  float ycurr = yprev + xydot.second * euler_dt;
  return std::pair<float, float>(xcurr, ycurr);
}

/**
 * \brief Uses Euler integation to obtain x,y from xdot, ydot, xprev, yprev. <br>!!!!! It diverges
 when euler_dt is too small !!!! <br> It would also be better to use Runge Kutta instead of Euler
 * \param float xprev : x angle at t-1
 * \param float yprev : y angle at t-1
 * \param std::pair<float, float> xydot : derivative of xprev, yprev obtained with computeXYdot
 * \param float euler_dt time step for euler integration : it is better to take less than 0.01
 * \return std::pair<float, float> return (x,y)
 */
std::pair<float, float> CPG::RK4(float xprev, float yprev, std::pair<float, float> xydot,
                                 float euler_dt) {
  float k1_x = xydot.first;
  float k2_x = xprev + k1_x * (euler_dt / 2);
  float k3_x = xprev + k2_x * (euler_dt / 2);
  float k4_x = xprev + k3_x * euler_dt;
  float xcurr = xprev + (euler_dt / 6) * (k1_x + 2 * k2_x + 2 * k3_x + k4_x);

  float k1_y = xydot.second;
  float k2_y = yprev + k1_y * (euler_dt / 2);
  float k3_y = yprev + k2_y * (euler_dt / 2);
  float k4_y = yprev + k3_y * euler_dt;
  float ycurr = yprev + (euler_dt / 6) * (k1_y + 2 * k2_y + 2 * k3_y + k4_y);

  return std::pair<float, float>(xcurr, ycurr);
}

} // namespace cpg
#endif
