#ifndef KALMANLOCALIZATION_H
#define KALMANLOCALIZATION_H

namespace kalman {

class KalmanLocalization {
 public:
  KalmanLocalization() {}
  ~KalmanLocalization() {}
  
 private:
  KalmanFilter filter;


}; // class KalmanLocalization

}; // namespace kalman

#endif

