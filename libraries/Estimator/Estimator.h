#include <AttitudeKalmanFilter.h>

class Estimator {
   public:
    Estimator();

    void init();
    void main();

   private:
    AttitudeKalmanFilter attitudeFilter;
}