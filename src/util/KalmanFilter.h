//
// Created by rolf on 05-11-19.
//

#ifndef RTT_KALMANFILTER_H
#define RTT_KALMANFILTER_H

#include "armadillo"

/*
 * A class for generic standard basic linear Kalman Filters using doubles and dense matrixes
 * @author Rolf
 * @param STATEDIM dimension of state vector
 * @param OBSDIM dimension of measurement vector
 */
template<int STATEDIM, int OBSDIM>
class KalmanFilter {
public:
    typedef arma::mat::fixed<STATEDIM, STATEDIM> Matrix;
    typedef arma::mat::fixed<OBSDIM, STATEDIM> MatrixO;
    typedef arma::mat::fixed<OBSDIM, OBSDIM> MatrixOO;
    typedef arma::mat::fixed<STATEDIM, OBSDIM> MatrixSO;
    typedef arma::vec::fixed<STATEDIM> Vector;
    typedef arma::vec::fixed<OBSDIM> VectorO;
private:
    //Variable names are the same as on Wikipedia: https://en.wikipedia.org/wiki/Kalman_filter

    Vector X;// State of the state
    Matrix P;// covariance matrix of the state (how sure are we of the state we predict?)

    //Same as above, but only used for prediction. We only save actual observations in X and P, and predictions here, generally.
    Vector Xtemp;
    Matrix Ptemp;
public:
    Matrix F;// Forward model/state update matrix. Essentially a linear model of what we predict the next state will be
    MatrixO H;// Observation model/ states how we can interpret observation as our state
    Matrix Q;// Covariance of the process noise. (Amount of "Random Forces" we can expect in the process)
    MatrixOO R;// Observation Noise Covariance. Keeps track of how noisy the observations are.
    VectorO z;// Observation
    //These are only really used in extended Kalman Filters
    Matrix B;// State transition jacobian
    Vector u;//Control input into the system (e.g. Robot Commands, thermostat)

    /*
     * Constructs a Kalman Filter which starts with initial values and noise estimates
     * By default we simply have every column/row independent and no noises anywhere
     */
    explicit KalmanFilter(const Vector& x, const Matrix& p) :
            X(x),
            Xtemp(x),
            P(p),
            Ptemp(p)
    {

        F.eye();
        H.zeros();
        Q.zeros();
        R.zeros();
        z.zeros();

        B.eye();
        u.zeros();

    };
    /*
     * @param permanentUpdate; if set to true, permanently updates the filter's state.
     * Otherwise, the prediction is only stored locally as a prediction
     */
    void predict(bool permanentUpdate) {
        Xtemp = F * X + u;
        Ptemp = B * P * B.t() + Q;
        if (permanentUpdate) {
            X = Xtemp;
            P = Ptemp;
        }
    };
    //TODO: use predicted states or old states?? Test/literature review
    /*
     * Updates the filter using a prediction.
     */
    void update() {
        VectorO y = z - (H * Xtemp);
        MatrixOO S = H * Ptemp * H.t() + R;
        MatrixSO K = Ptemp * H.t() * S.i();
        X = Xtemp + K * y;
        Matrix Identity;
        Identity.eye();
        P = (Identity - K * H) * Ptemp;
    };

    const Vector& state() const{
        return Xtemp;
    }
    const Vector& basestate() const{
        return X;
    }

    void modifyState(int index, double value){
        Xtemp.at(index) = value;
    }

};

#endif //RTT_KALMANFILTER_H
