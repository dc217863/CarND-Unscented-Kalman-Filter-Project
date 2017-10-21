#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    std::cout << "UKF Constructor" << std::endl;
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.5;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 1.5;

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;

    /**
     TODO:

    Complete the initialization. See ukf.h for other member properties.

    Hint: one or more values initialized above might be wildly off...
    */
    is_initialized_ = false;
    
    n_x_ = 5;
    
    n_aug_ = 7;

    // rho, phi, rhod
    int n_z_radar = 3;

    // px and py
    int n_z_laser = 2;
    
    lambda_ = 3 - n_aug_;

    Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

    weights_ = VectorXd(2*n_aug_+1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage &meas_package) {
    /**
     TODO:

    Complete this function! Make sure you switch between lidar and radar
    measurements.
    */
    std::cout << "UKF ProcessMeasurement" << std::endl;
    if (!is_initialized_) {
        x_ << 0, 0, 0, 0, 0;

        P_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0,
              0, 0, 1000, 0, 0,
              0, 0, 0, 1000, 0,
              0, 0, 0, 0, 1000;
        
        if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR) {
            std::cout << "UKF PM init radar" << std::endl;
            x_(0) = meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(1));
            x_(1) = meas_package.raw_measurements_(0) * sin(meas_package.raw_measurements_(1));
            x_(2) = 0;
            x_(3) = 0;
            x_(4) = 0;

        } else if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER) {
            std::cout << "UKF PM init laser" << std::endl;
            x_(0) = meas_package.raw_measurements_(0);
            x_(1) = meas_package.raw_measurements_(1);
            x_(2) = 0;
            x_(3) = 0;
            x_(4) = 0;
        }
        is_initialized_ = true;
        time_us_ = meas_package.timestamp_;
        Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
        std::cout << "UKF PM init over" << std::endl;
    }
    float delta_t = 
    (meas_package.timestamp_ - time_us_) / 1000000.0; // in seconds
    time_us_ = meas_package.timestamp_;
    Prediction(delta_t);
    // std::cout << "1x_: " << x_ << std::endl;
    std::cout << "1P_: " << P_ << std::endl;
    if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR) {
        UpdateRadar(meas_package);
    } else if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER) {
        UpdateLidar(meas_package);
    }
    // std::cout << "2x_: " << x_ << std::endl;
    std::cout << "2P_: " << P_ << std::endl;
    std::cout << "-------------------" << std::endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**
     TODO:

    Complete this function! Estimate the object's location. Modify the state
    vector, x_. Predict sigma points, the state, and the state covariance matrix.
    */
    std::cout << "UKF Prediction" << std::endl;
    auto Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
    AugmentedSigmaPoints(&Xsig_aug);

    std::cout << "!!1 Xsig_pred_: " << Xsig_pred_ << std::endl;
    SigmaPointPrediction(Xsig_aug, delta_t);
    std::cout << "!!2 Xsig_pred_: " << Xsig_pred_ << std::endl;

    PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
    TODO:
  
    Complete this function! Use lidar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.
  
    You'll also need to calculate the lidar NIS.
    */
    std::cout << "UKF UpdateLidar" << std::endl;
  }

  /**
   * Updates the state and the state covariance matrix using a radar measurement.
   * @param {MeasurementPackage} meas_package
   */
  void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
    TODO:
  
    Complete this function! Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.
  
    You'll also need to calculate the radar NIS.
    */
    std::cout << "UKF UpdateRadar" << std::endl;

    VectorXd Z_pred = VectorXd(n_z_radar);
    MatrixXd S = MatrixXd(n_z_radar, n_z_radar);
    
    //create matrix for sigma points in measurement space
    auto Zsig = MatrixXd(n_z_radar, 2 * n_aug_ + 1);
    
    PredictRadarMeasurement(&Z_pred, &S, &Zsig);

    UpdateState(meas_package, Z_pred, S, Zsig);
  }

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_aug) {
    std::cout << "UKF AugmentedSigmaPoints" << std::endl;
    //create sigma point matrix
    auto Xsig_out = MatrixXd(n_aug_, 2*n_aug_+1);
    
    //create augmented state covariance
    auto P_aug = MatrixXd(n_aug_, n_aug_);
    
    Xsig_out.col(0).head(n_x_) = x_;
    
    auto Q = MatrixXd(2,2);
    Q << std_a_*std_a_, 0,
         0, std_yawdd_*std_yawdd_;
    
    P_aug.topLeftCorner(n_x_,n_x_) = P_;
    P_aug.bottomRightCorner(2,2) = Q;
    
    MatrixXd A_aug = P_aug.llt().matrixL();
    
    for (int i = 1; i < n_aug_ + 1; i++) {
        Xsig_out.col(i) = Xsig_out.col(0) + sqrt(lambda_ + n_aug_) * A_aug.col(i-1);
        Xsig_out.col(i+7) = Xsig_out.col(0) - sqrt(lambda_ + n_aug_) * A_aug.col(i-1);
    }
    *Xsig_aug = Xsig_out;
}

void UKF::SigmaPointPrediction(const MatrixXd& Xsig_aug, double delta_t) {
    std::cout << "UKF SigmaPointPrediction" << std::endl;
    //create matrix with predicted sigma points as columns
    auto Xsig_out = MatrixXd(n_x_, 2 * n_aug_ + 1);

    Xsig_out = Xsig_aug.block(0, 0, n_x_, 2*n_aug_+1);

    for (int i=0; i<2*n_aug_+1; i++) {
        double v = Xsig_aug(2,i);
        double psi = Xsig_aug(3,i);
        double psid = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_psidd = Xsig_aug(6,i);
        
        double px_1, py_1, psi_1;
        if (fabs(psid) > 0.001) {
              px_1 = (v / psid) * (sin(psi + psid * delta_t) - sin(psi));
              py_1 = (v / psid) * (cos(psi) - cos(psi + psid * delta_t));
              psi_1 = psid * delta_t;
        } else {
              px_1 = v * cos(psi) * delta_t;
              py_1 = v * sin(psi) * delta_t;
              psi_1 = 0;
        }
        
        double px_2 = 0.5 * delta_t * delta_t * cos(psi) * nu_a;
        double py_2 = 0.5 * delta_t * delta_t * sin(psi) * nu_a;
        double v_2 = delta_t * nu_a;
        double psi_2 = 0.5 * delta_t * delta_t * nu_psidd;
        double psid_2 = delta_t * nu_psidd;
  
        Xsig_out(0,i) = Xsig_out(0,i) + px_1 + px_2;
        Xsig_out(1,i) = Xsig_out(1,i) + py_1 + py_2;
        Xsig_out(2,i) = Xsig_out(2,i) + v_2;
        Xsig_out(3,i) = Xsig_out(3,i) + psi_1 + psi_2;
        Xsig_out(4,i) = Xsig_out(4,i) + psid_2;
    }
    Xsig_pred_ = Xsig_out;
}

void UKF::PredictMeanAndCovariance() {
    std::cout << "UKF PredictMeanAndCovariance" << std::endl;
    auto x = VectorXd(n_x_);
    auto P = MatrixXd(n_x_, n_x_);

    x.fill(0.0);
    for (int i=0; i<2*n_aug_+1; i++) {
        double w_i;
        if (i==0) {
            w_i = lambda_ / (lambda_ + n_aug_);
        } else {
            w_i = 1 / (2 * (lambda_ + n_aug_));
        }
        weights_(i) = w_i;
        
        x = x + (w_i * Xsig_pred_.col(i));
    }
    
    P.fill(0.0);
    std::cout << "Xsig_pred_: " << Xsig_pred_ << std::endl;
    for (int i=0; i<2*n_aug_+1; i++) {
        VectorXd X_diff = Xsig_pred_.col(i) - x;
        //angle normalization
        while (X_diff(3)> M_PI) X_diff(3)-=2.*M_PI;
        while (X_diff(3)<-M_PI) X_diff(3)+=2.*M_PI;
        
        P = P + weights_(i) * X_diff * X_diff.transpose();
        // std::cout << "!!P " << i << ": " << P << std::endl;
    }
    x_ = x;
    P_ = P;
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Z_out) {
    std::cout << "UKF PredictRadarMeasurement" << std::endl;
    // auto Z_sig = MatrixXd(n_z_radar, 2 * n_aug_ + 1);
    auto Z_sig = MatrixXd(n_z_radar, 2 * n_aug_ + 1);
    //mean predicted measurement
    auto z_pred = VectorXd(n_z_radar);

    //measurement covariance matrix S
    auto S = MatrixXd(n_z_radar,n_z_radar);
    
    for(int i=0;i<2*n_aug_+1; i++) {
        double px = Xsig_pred_(0,i);
        double py = Xsig_pred_(1,i);
        double v = Xsig_pred_(2,i);
        double psi = Xsig_pred_(3,i);
        
        double rho = sqrt(px*px + py*py);
        double phi = atan2(py, px);
        double rhod = (px * cos(psi) * v + py * sin(psi) * v) / rho;
        
        Z_sig(0,i) = rho;
        Z_sig(1,i) = phi;
        Z_sig(2,i) = rhod;
    }
    
    z_pred.fill(0.0);
    for (int i=0; i<2*n_aug_+1; i++) {
        double w_i;
        if (i==0) {
            w_i = lambda_ / (lambda_ + n_aug_);
        } else {
            w_i = 1 / (2 * (lambda_ + n_aug_));
        }
        weights_(i) = w_i;
        
        z_pred = z_pred + (w_i * Z_sig.col(i));
    }
    
    S.fill(0.0);
    for (int i=0; i<2*n_aug_+1; i++) {
        MatrixXd z_pred_diff = Z_sig.col(i) - z_pred;
        S = S + weights_(i) * z_pred_diff * z_pred_diff.transpose();
    }
    
    auto R = MatrixXd(n_z_radar, n_z_radar);
    R.fill(0.0);
    R(0,0) = std_radr_ * std_radr_;
    R(1,1) = std_radphi_ * std_radphi_;
    R(2,2) = std_radrd_ * std_radrd_;
    S = S + R;

    *z_out = z_pred;
    *S_out = S;
    *Z_out = Z_sig;
}

void UKF::UpdateState(MeasurementPackage meas_package, const VectorXd& Z_pred, const MatrixXd& S, const MatrixXd& Zsig) {
    std::cout << "UKF UpdateState" << std::endl;
    // measurement at k+1
    VectorXd z = VectorXd(n_z_radar);
    //TODO: check correctness
    z(0) = meas_package.raw_measurements_(0);
    z(1) = meas_package.raw_measurements_(1);
    z(2) = meas_package.raw_measurements_(2);

    //create matrix for cross correlation Tc
    auto Tc = MatrixXd(n_x_, n_z_radar);

    for (int i=0; i<2*n_aug_+1; i++) {
        //residual
        VectorXd z_diff = Zsig.col(i) - Z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
      }
      
      MatrixXd K = Tc * S.inverse();
      
      VectorXd z_diff = z - Z_pred;
      //angle normalization
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
      
      x_ = x_ + K * z_diff;
      P_ = P_ - K * S * K.transpose();
}
