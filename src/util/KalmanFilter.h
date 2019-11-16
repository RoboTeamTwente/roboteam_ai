#ifndef RTT_KALMANFILTER_H
#define RTT_KALMANFILTER_H

#include "armadillo"

/**
 * A class for generic standard basic linear Kalman Filters using doubles and dense matrixes.
 * Variable names are the same as on wikipedia: https://en.wikipedia.org/wiki/Kalman_filter
 * @author Rolf
 * @date 5 November 2019
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
    Vector X;// State of the system
    Matrix P;// covariance matrix of the system (how sure are we of the state?)

    //Same as above, but only used for prediction. We only save actual observations in X and P, and predictions here, generally.
    Vector Xtemp;
    Matrix Ptemp;
public:
    Matrix F;// Forward model/state update matrix. Essentially a linear model of what we predict the next state will be
    MatrixO H;// Observation model/ states how we can interpret observation as our state
    Matrix Q;// Covariance of the process noise. (Amount of "Random Forces" we can expect in the process)
    MatrixOO R;// Observation Noise Covariance. Keeps track of how noisy the observations are.
    VectorO z;// Observation itself.

    //These are only really used in extended Kalman Filters or when we add control input.
    Matrix B;// State transition jacobian
    Vector u;//Control input into the system (e.g. Robot Commands, thermostat)

    /**
     * Constructs a Kalman Filter which starts with initial values and noise estimates
     * By default we simply have every column/row independent and no noises anywhere
     * @param x initial state vector
     * @param p initial covariance vector
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
    /**
     * Predict the next state using forward model, updating the state and covariance estimates
     * @param permanentUpdate if set to true, permanently updates the filter's state.
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

    /**
     * Updates the filter using the current observation z that is set
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

    /**
     * Returns the state of the system. Does also include predictions.
     * @return (predicted) state of the system
     */
    const Vector& state() const{
        return Xtemp;
    }

    /**
     * Returns the state of the system. Only includes permanent Updates and observations.
     * @return State of the system only based on observations (no forward prediction)
     */
    const Vector& basestate() const{
        return X;
    }

    /**
     * Manually set state[index]=value, modifying the state of the filter. Should only be used sparsely.
     * @param index. Index of the state to be modified
     * @param value. Value to set state[index] to
     */
    void modifyState(int index, double value){
        Xtemp.at(index) = value;
    }

};

#endif //RTT_KALMANFILTER_H
