#pragma once

#include "Particle.h"
#include <Eigen/Dense>;
using namespace Eigen;

class SpringForce {
 public:
  SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd);

  void draw();
  void calculateForce();
  std::vector<Particle *> getParticles();

  MatrixXf Jx = MatrixXf::Zero(2, 2);
  MatrixXf Jv = MatrixXf::Zero(2, 2);

  double const m_dist;     // rest length
  double const m_ks, m_kd; // spring strength constants

 private:

  Particle * const m_p1;   // particle 1
  Particle * const m_p2;   // particle 2 
};
