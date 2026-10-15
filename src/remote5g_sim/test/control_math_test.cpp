#include <remote5g_sim/control_math.hpp>
#include <limits>
#include <stdexcept>
#include <iostream>
int main() {
  auto check=[](bool ok) {if(!ok) throw std::runtime_error("control math regression");};
  auto l=remote5g::ackermann(0.3,0.23861,0.24),r=remote5g::ackermann(-0.3,0.23861,0.24);
  check(l.first>l.second && l.second>0); check(r.second<r.first && r.first<0);
  check(std::abs(l.first+r.second)<1e-12); check(remote5g::ackermann(0,0.23861,0.24).first==0);
  double a=0; check(!remote5g::twist(0,1,0.23861,a)); check(remote5g::twist(-0.2,0.3,0.23861,a)&&a<0);
  check(!remote5g::twist(std::numeric_limits<double>::quiet_NaN(),0,0.23861,a));
  check(!remote5g::twist(0,std::numeric_limits<double>::infinity(),0.23861,a));
  check(remote5g::speed_step(0.2,-0.2,0.8,1.2,0.05)>0);
  check(std::abs(remote5g::speed_step(0,0.6,0.8,1.2,0.05)-0.04)<1e-12);
  std::cout<<"Geometry, reverse Twist, zero-speed rejection, NaN/Inf and ramp checks passed\n";
}
