#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <random>
#include <chrono>
#include <fstream>


Eigen::Matrix<float, 7, 1> get_random_matrix(Eigen::Matrix<float, 7, 1> mean, Eigen::Matrix<float, 7, 7> Cov) {

  Eigen::LLT<Eigen::Matrix<float, 7, 7>> LLT(Cov);

  if (LLT.info() != Eigen::Success) {
    throw std::runtime_error("Covariance matrix is not positive definite!");
  }

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();   
  std::default_random_engine generator(seed);
  std::normal_distribution<double> dist(0.0, 1.0);


  Eigen::Matrix<float, 7, 1> Rand;

  for (int i = 0; i < 7; i++) {
    Rand(i,0) = dist(generator);
  }

  return (mean + (LLT.matrixL() * Rand));
}


class Projectile {
public:
  Eigen::Matrix<float, 7, 1> State;
  float per_;

  Eigen::Matrix<float, 7, 7> A;

  Projectile(float x, float y, float z, float vx, float vy, float vz, float g, float period) {
    State(0,0) = x;
    State(1,0) = y;
    State(2,0) = x;
    State(3,0) = vx;
    State(4,0) = vy;
    State(5,0) = vz;
    State(6,0) = g;
    per_ = period;

    A << 1, 0, 0, per_, 0, 0, 0, \
      0, 1, 0, 0, per_, 0, 0, \
      0, 0, 1, 0, 0, per_, -(per_*per_), \
      0, 0, 0, 1, 0, 0, 0, \
      0, 0, 0, 0, 1, 0, 0, \
      0, 0, 0, 0, 0, 1, -per_, \
      0, 0, 0, 0, 0, 0, 1 ;
  }

  
  void tick() {
  Eigen::Matrix<float, 7, 1> m;
  m << 0, 0, 0, 0, 0, 0, 0;

  
  Eigen::Matrix<float, 7, 7> cov;
  // cov <<								\
  //   0.01, 0.005,   0.005,    0,    0,    0,    0,			\
  //   0.005,   0.01, 0.005,    0,    0,    0,    0,			\
  //   0.005,   0.005,   0.01,  0,    0,    0,    0,			\
  //   0,   0,   0,    0.01,  0.005, 0.005, 0,				\
  //   0,   0,   0,    0.005, 0.01,  0.005, 0,				\
  //   0,   0,   0,    0.005, 0.005, 0.01,  0,				\
  //   0,   0,   0,    0,    0,    0,    0.00001				\
  //   ;

  cov <<								\
    0.01, 0.0000005,   0.0000005,    0,    0,    0,    0,			\
    0.0000005,   0.01, 0.0000005,    0,    0,    0,    0,			\
    0.0000005,   0.0000005,   0.01,  0,    0,    0,    0,			\
    0,   0,   0,    0.00000001,  0.00, 0.0000, 0,				\
    0,   0,   0,    0.0000, 0.00000001,  0.0000, 0,				\
    0,   0,   0,    0.0000, 0.0000, 0.0000001,  0,				\
    0,   0,   0,    0,    0,    0,    0.00001				\
    ;




    State = (A*State) + get_random_matrix(m, cov);;
  }

};


std::ostream &operator<<(std::ostream &os, Projectile p) {
  return os << p.State(0,0) << "    " << p.State(1,0) << "    " << p.State(2,0);
}


class Kalman {
public:
  Eigen::Matrix<float, 7, 1> State;
  float per_;
  
  Eigen::Matrix<float, 7, 7> A;
  
  Eigen::Matrix<float, 3, 7> H;




  Eigen::Matrix<float, 3, 3> cov_noise_sensor;

  Eigen::Matrix<float, 7, 7> P;

  Kalman(float x, float y, float z, float vx, float vy, float vz, float g, float period) {
    State(0,0) = x;
    State(1,0) = y;
    State(2,0) = x;
    State(3,0) = vx;
    State(4,0) = vy;
    State(5,0) = vz;
    State(6,0) = g;
    per_ = period;
    
    A << 1, 0, 0, per_, 0, 0, 0,		\
      0, 1, 0, 0, per_, 0, 0,			\
      0, 0, 1, 0, 0, per_, -(per_*per_),	\
      0, 0, 0, 1, 0, 0, 0,			\
      0, 0, 0, 0, 1, 0, 0,			\
      0, 0, 0, 0, 0, 1, -per_,			\
      0, 0, 0, 0, 0, 0, 1 ;

    H << \
      1, 0, 0, 0, 0, 0, 0, \
      0, 1, 0, 0, 0, 0, 0, \
      0, 0, 1, 0, 0, 0, 0 \
      ;

    cov_noise_sensor <<		
      0.1, 0.05,   0.05,
      0.05,   0.1, 0.05,
      0.05,   0.005,  0.1 ;
    

    P << \
      1, 0, 0, 0, 0, 0, 0,  \
      0, 1, 0, 0, 0, 0, 0,  \
      0, 0, 1, 0, 0, 0, 0,  \
      0, 0, 0, 1, 0, 0, 0,  \
      0, 0, 0, 0, 1, 0, 0,  \
      0, 0, 0, 0, 0, 1, 0,  \
      0, 0, 0, 0, 0, 0, 1  \
      ;




  }
  
  
  void predict() {
    State = A*State;
    P = A*P*A.transpose();
  }

  void update(Eigen::Matrix<float, 3, 1> measurement) {

    Eigen::Matrix<float, 3, 1> ybar = measurement - H*State;
    Eigen::Matrix<float, 3, 3> S = H*P*H.transpose() + cov_noise_sensor;
    Eigen::Matrix<float, 7, 3> K = P*H.transpose()*S.inverse();

    State = State + K*ybar;

    P = P - K*H*P;

  }

  Kalman return_state() {
    return *this;
  }
  
};


std::ostream &operator<<(std::ostream &os, Kalman p) {
  return os << p.State(0,0) << "    " << p.State(1,0) << "    " << p.State(2,0);
}


int main() {
  //           x     y    z    vx  vy    vz     g     T
  Projectile P(0.0, 0.0, 0.0, 5.0, 5.0, 10.0, 9.81, 0.010);
  Kalman F(0.0, 0.0, 0.0, 4.0, 7.0, 15.0, 9.81, 0.010);

  std::ofstream fs("data.txt", std::ios::out);


  // std::cout << get_random_matrix(m, cov) << std::endl;


  
  Eigen::Matrix<float, 3, 1> tosend;

  for (int i=0; i<1000; i++) {
    if (P.State(2,0) < 0) break;
    fs << P << "    "; // << std::endl;
    tosend(0,0) = P.State(0,0);
    tosend(1,0) = P.State(1,0);
    tosend(2,0) = P.State(2,0);
    F.predict();
    fs << F;
    fs <<  std::endl;
    F.update(tosend);
    P.tick();

  }

}
  
  
