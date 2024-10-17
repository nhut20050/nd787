#include "Common.h"
#include "QuadEstimatorEKF.h"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"
#include "Math/Quaternion.h"

using namespace SLR;

const int QuadEstimatorEKF::QUAD_EKF_NUM_STATES;

QuadEstimatorEKF::QuadEstimatorEKF(string config, string name)
    : BaseQuadEstimator(config),
      Q(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES),
      R_GPS(6, 6),
      R_Mag(1, 1),
      ekfState(QUAD_EKF_NUM_STATES),
      ekfCov(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES),
      trueError(QUAD_EKF_NUM_STATES)
{
  _name = name;
  Init();
}

QuadEstimatorEKF::~QuadEstimatorEKF()
{
}

void QuadEstimatorEKF::Init()
{
  ParamsHandle paramSys = SimpleConfig::GetInstance();

  paramSys->GetFloatVector(_config + ".InitState", ekfState);

  VectorXf initStdDevs(QUAD_EKF_NUM_STATES);
  paramSys->GetFloatVector(_config + ".InitStdDevs", initStdDevs);
  ekfCov.setIdentity();
  for (int i = 0; i < QUAD_EKF_NUM_STATES; i++)
  {
    ekfCov(i, i) = initStdDevs(i) * initStdDevs(i);
  }

  // complementary filter params
  attitudeTau = paramSys->Get(_config + ".AttitudeTau", .1f);
  dtIMU = paramSys->Get(_config + ".dtIMU", .002f);

  pitchEst = 0;
  rollEst = 0;

  // GPS measurement model covariance
  R_GPS.setZero();
  R_GPS(0, 0) = R_GPS(1, 1) = powf(paramSys->Get(_config + ".GPSPosXYStd", 0), 2);
  R_GPS(2, 2) = powf(paramSys->Get(_config + ".GPSPosZStd", 0), 2);
  R_GPS(3, 3) = R_GPS(4, 4) = powf(paramSys->Get(_config + ".GPSVelXYStd", 0), 2);
  R_GPS(5, 5) = powf(paramSys->Get(_config + ".GPSVelZStd", 0), 2);

  // magnetometer measurement model covariance
  R_Mag.setZero();
  R_Mag(0, 0) = powf(paramSys->Get(_config + ".MagYawStd", 0), 2);

  // load the transition model covariance
  Q.setZero();
  Q(0, 0) = Q(1, 1) = powf(paramSys->Get(_config + ".QPosXYStd", 0), 2);
  Q(2, 2) = powf(paramSys->Get(_config + ".QPosZStd", 0), 2);
  Q(3, 3) = Q(4, 4) = powf(paramSys->Get(_config + ".QVelXYStd", 0), 2);
  Q(5, 5) = powf(paramSys->Get(_config + ".QVelZStd", 0), 2);
  Q(6, 6) = powf(paramSys->Get(_config + ".QYawStd", 0), 2);
  Q *= dtIMU;

  rollErr = pitchErr = maxEuler = 0;
  posErrorMag = velErrorMag = 0;
}

void QuadEstimatorEKF::UpdateFromIMU(V3F accel, V3F gyro)
{
  // Improve a complementary filter-type attitude filter
  //
  // Currently a small-angle approximation integration method is implemented
  // The integrated (predicted) value is then updated in a complementary filter style with attitude information from accelerometers
  //
  // Implement a better integration method that uses the current attitude estimate (rollEst, pitchEst and ekfState(6))
  // to integrate the body rates into new Euler angles.
  //
  // HINTS:
  //  - there are several ways to go about this, including:
  //    1) create a rotation matrix based on your current Euler angles, integrate that, convert back to Euler angles
  //    OR
  //    2) use the Quaternion<float> class, which has a handy FromEuler123_RPY function for creating a quaternion from Euler Roll/PitchYaw
  //       (Quaternion<float> also has a IntegrateBodyRate function, though this uses quaternions, not Euler angles)

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Declaring a quaternion for the current orientation
  Quaternion<float> quat;
  // Declaring a quaternion for the change in orientation (body rates)
  Quaternion<float> dq;

  // Convert the estimated roll, pitch, and current yaw (from ekfState) to a quaternion
  quat = quat.FromEuler123_RPY(rollEst, pitchEst, ekfState(6));

  // Integrate body rates from gyroscope to update quaternion and get the new orientation
  Quaternion<float> quat_pqr = dq.IntegrateBodyRate(gyro, dtIMU) * quat;

  // Extract predicted pitch angle from the updated quaternion
  float predictedPitch = quat_pqr.Pitch();
  // Extract predicted roll angle from the updated quaternion
  float predictedRoll = quat_pqr.Roll();
  // Update the yaw angle in the state vector from the updated quaternion
  ekfState(6) = quat_pqr.Yaw();

  // Normalize yaw to ensure it stays within the range [-π, π]
  if (ekfState(6) > F_PI)
    ekfState(6) -= 2.f * F_PI; // Adjust yaw if it's greater than π
  if (ekfState(6) < -F_PI)
    ekfState(6) += 2.f * F_PI; // Adjust yaw if it's less than -π

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  // CALCULATE UPDATE
  accelRoll = atan2f(accel.y, accel.z);
  accelPitch = atan2f(-accel.x, 9.81f);

  // FUSE INTEGRATION AND UPDATE
  rollEst = attitudeTau / (attitudeTau + dtIMU) * (predictedRoll) + dtIMU / (attitudeTau + dtIMU) * accelRoll;
  pitchEst = attitudeTau / (attitudeTau + dtIMU) * (predictedPitch) + dtIMU / (attitudeTau + dtIMU) * accelPitch;

  lastGyro = gyro;
}

void QuadEstimatorEKF::UpdateTrueError(V3F truePos, V3F trueVel, Quaternion<float> trueAtt)
{
  VectorXf trueState(QUAD_EKF_NUM_STATES);
  trueState(0) = truePos.x;
  trueState(1) = truePos.y;
  trueState(2) = truePos.z;
  trueState(3) = trueVel.x;
  trueState(4) = trueVel.y;
  trueState(5) = trueVel.z;
  trueState(6) = trueAtt.Yaw();

  trueError = ekfState - trueState;
  if (trueError(6) > F_PI)
    trueError(6) -= 2.f * F_PI;
  if (trueError(6) < -F_PI)
    trueError(6) += 2.f * F_PI;

  pitchErr = pitchEst - trueAtt.Pitch();
  rollErr = rollEst - trueAtt.Roll();
  maxEuler = MAX(fabs(pitchErr), MAX(fabs(rollErr), fabs(trueError(6))));

  posErrorMag = truePos.dist(V3F(ekfState(0), ekfState(1), ekfState(2)));
  velErrorMag = trueVel.dist(V3F(ekfState(3), ekfState(4), ekfState(5)));
}

VectorXf QuadEstimatorEKF::PredictState(VectorXf curState, float dt, V3F accel, V3F gyro)
{
  assert(curState.size() == QUAD_EKF_NUM_STATES);
  VectorXf predictedState = curState;
  // Predict the current state forward by time dt using current accelerations and body rates as input
  // INPUTS:
  //   curState: starting state
  //   dt: time step to predict forward by [s]
  //   accel: acceleration of the vehicle, in body frame, *not including gravity* [m/s2]
  //   gyro: body rates of the vehicle, in body frame [rad/s]
  //
  // OUTPUT:
  //   return the predicted state as a vector

  // HINTS
  // - dt is the time duration for which you should predict. It will be very short (on the order of 1ms)
  //   so simplistic integration methods are fine here
  // - we've created an Attitude Quaternion for you from the current state. Use
  //   attitude.Rotate_BtoI(<V3F>) to rotate a vector from body frame to inertial frame
  // - the yaw integral is already done in the IMU update. Be sure not to integrate it again here

  Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, curState(6));

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // The implementation below follows document section 7.2 for the Estimator
  Mat3x3F rotationMatrixB = attitude.RotationMatrix_IwrtB(); // Get rotation matrix from body frame to inertial frame

  // Create a matrix B to store state transition values
  MatrixXf stateTransitionMatrix(QUAD_EKF_NUM_STATES, 4); // 7 states and 4 inputs
  stateTransitionMatrix.row(0) << 0, 0, 0, 0;             // Placeholder for state 0 (unused)
  stateTransitionMatrix.row(1) << 0, 0, 0, 0;             // Placeholder for state 1 (unused)
  stateTransitionMatrix.row(2) << 0, 0, 0, 0;             // Placeholder for state 2 (unused)
  // Fill in the rotational components from the rotation matrix for state 3
  stateTransitionMatrix.row(3) << rotationMatrixB(0, 0), rotationMatrixB(0, 1), rotationMatrixB(0, 2), 0;
  // Fill in the rotational components from the rotation matrix for state 4
  stateTransitionMatrix.row(4) << rotationMatrixB(1, 0), rotationMatrixB(1, 1), rotationMatrixB(1, 2), 0;
  // Fill in the rotational components from the rotation matrix for state 5
  stateTransitionMatrix.row(5) << rotationMatrixB(2, 0), rotationMatrixB(2, 1), rotationMatrixB(2, 2), 0;
  stateTransitionMatrix.row(6) << 0, 0, 0, 1; // Placeholder for state 6 (control input)

  // Create a vector to hold the updated state values
  VectorXf updatedState(QUAD_EKF_NUM_STATES);
  updatedState << curState[0] + dt * curState[3], // Update position x
      curState[1] + dt * curState[4],             // Update position y
      curState[2] + dt * curState[5],             // Update position z
      curState[3],                                // Retain velocity x
      curState[4],                                // Retain velocity y
      curState[5] - (9.81f * dt),                 // Update velocity z (gravity effect)
      curState[6];                                // Retain the state (e.g., yaw)

  // Create a control input vector for acceleration
  VectorXf controlInput(4);
  controlInput << accel.x, // Acceleration in x direction
      accel.y,             // Acceleration in y direction
      accel.z,             // Acceleration in z direction
      0.0;                 // Placeholder for the fourth input (unused)

  // Calculate the predicted state by adding the updated state and the effect of control inputs
  predictedState = updatedState + (stateTransitionMatrix * (controlInput * dt));

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return predictedState;
}

MatrixXf QuadEstimatorEKF::GetRbgPrime(float roll, float pitch, float yaw)
{
  // first, figure out the Rbg_prime
  MatrixXf RbgPrime(3, 3);
  RbgPrime.setZero();

  // HINTS
  // - this is just a matter of putting the right sin() and cos() functions in the right place.
  //   make sure you write clear code and triple-check your math
  // - You can also do some numerical partial derivatives in a unit test scheme to check
  //   that your calculations are reasonable

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Defining variables for yaw, pitch, and roll angles
  float yawAngle = yaw;     // Yaw angle (psi)
  float pitchAngle = pitch; // Pitch angle (phi)
  float rollAngle = roll;   // Roll angle (theta)

  // Creating the rotation matrix RbgPrime to represent the orientation transformation
  RbgPrime(0, 0) = -cos(rollAngle) * sin(yawAngle);                                                     // Row 0, Column 0
  RbgPrime(0, 1) = -sin(pitchAngle) * sin(rollAngle) * sin(yawAngle) - cos(pitchAngle) * cos(yawAngle); // Row 0, Column 1
  RbgPrime(0, 2) = -cos(pitchAngle) * sin(rollAngle) * sin(yawAngle) + sin(pitchAngle) * cos(yawAngle); // Row 0, Column 2
  RbgPrime(1, 0) = cos(rollAngle) * cos(yawAngle);                                                      // Row 1, Column 0
  RbgPrime(1, 1) = sin(pitchAngle) * sin(rollAngle) * cos(yawAngle) - cos(pitchAngle) * sin(yawAngle);  // Row 1, Column 1
  RbgPrime(1, 2) = cos(pitchAngle) * sin(rollAngle) * cos(yawAngle) + sin(pitchAngle) * sin(yawAngle);  // Row 1, Column 2
  RbgPrime(2, 0) = 0;                                                                                   // Row 2, Column 0 (Unused or zero row)
  RbgPrime(2, 1) = 0;                                                                                   // Row 2, Column 1 (Unused or zero row)
  RbgPrime(2, 2) = 0;                                                                                   // Row 2, Column 2 (Unused or zero row)

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return RbgPrime;
}

void QuadEstimatorEKF::Predict(float dt, V3F accel, V3F gyro)
{
  // predict the state forward
  VectorXf newState = PredictState(ekfState, dt, accel, gyro);

  // HINTS
  // - update the covariance matrix cov according to the EKF equation.
  //
  // - you may find the current estimated attitude in variables rollEst, pitchEst, state(6).
  //
  // - use the class MatrixXf for matrices. To create a 3x5 matrix A, use MatrixXf A(3,5).
  //
  // - the transition model covariance, Q, is loaded up from a parameter file in member variable Q
  //
  // - This is unfortunately a messy step. Try to split this up into clear, manageable steps:
  //   1) Calculate the necessary helper matrices, building up the transition jacobian
  //   2) Once all the matrices are there, write the equation to update cov.
  //
  // - if you want to transpose a matrix in-place, use A.transposeInPlace(), not A = A.transpose()
  //

  // partial derivative of the Rbg matrix
  MatrixXf RbgPrime = GetRbgPrime(rollEst, pitchEst, ekfState(6));

  // we've created an empty Jacobian for you, currently simply set to identity
  MatrixXf gPrime(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES);
  gPrime.setIdentity();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Predict the current covariance forward by dt using the current accelerations and body rates as input.
  // INPUTS:
  //   dt: time step to predict forward by [s]
  //   accel: acceleration of the vehicle, in body frame, *not including gravity* [m/s2]
  //   gyro: body rates of the vehicle, in body frame [rad/s]
  //   state (member variable): current state (state at the beginning of this prediction)
  //
  // OUTPUT:
  //   update the member variable cov to the predicted covariance

  // Creating the matrix gPrime with identity components
  gPrime(0, 0) = 1; // Set element (0,0) to 1
  gPrime(1, 1) = 1; // Set element (1,1) to 1
  gPrime(2, 2) = 1; // Set element (2,2) to 1
  gPrime(3, 3) = 1; // Set element (3,3) to 1
  gPrime(4, 4) = 1; // Set element (4,4) to 1
  gPrime(5, 5) = 1; // Set element (5,5) to 1
  gPrime(6, 6) = 1; // Set element (6,6) to 1

  // Set the time delta for state transitions
  gPrime(0, 3) = dt; // Time step for state 0
  gPrime(1, 4) = dt; // Time step for state 1
  gPrime(2, 5) = dt; // Time step for state 2

  // Create a 3x3 rotation matrix from the estimated roll, pitch, and yaw
  MatrixXf rotationMatrix(3, 3);
  rotationMatrix = GetRbgPrime(rollEst, pitchEst, newState(6)); // Get rotation matrix based on estimated angles

  // Compute the contributions from acceleration transformed by the rotation matrix for each axis (x, y, z)
  gPrime(3, 6) = dt * (accel.x * rotationMatrix(0, 0) + accel.y * rotationMatrix(0, 1) + accel.z * rotationMatrix(0, 2)); // Contribution to state 3
  gPrime(4, 6) = dt * (accel.x * rotationMatrix(1, 0) + accel.y * rotationMatrix(1, 1) + accel.z * rotationMatrix(1, 2)); // Contribution to state 4
  gPrime(5, 6) = dt * (accel.x * rotationMatrix(2, 0) + accel.y * rotationMatrix(2, 1) + accel.z * rotationMatrix(2, 2)); // Contribution to state 5

  // Create a transposed version of gPrime for later calculations
  MatrixXf gPrimeTranspose(7, 7);
  gPrimeTranspose = gPrime.transpose(); // Transpose gPrime matrix

  // Update the EKF covariance with process noise Q
  ekfCov = gPrime * ekfCov * gPrimeTranspose + Q; // Propagate covariance and add process noise

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  ekfState = newState;
}

void QuadEstimatorEKF::UpdateFromGPS(V3F pos, V3F vel)
{
  // setting up vector with 6X6 values
  VectorXf z(6), zFromX(6);
  z(0) = pos.x;
  z(1) = pos.y;
  z(2) = pos.z;
  z(3) = vel.x;
  z(4) = vel.y;
  z(5) = vel.z;
  // create the matrix
  MatrixXf hPrime(6, QUAD_EKF_NUM_STATES);
  hPrime.setZero();

  // GPS UPDATE
  // Hints:
  //  - The GPS measurement covariance is available in member variable R_GPS
  //  - this is a very simple update
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Setting the "diagonal" elements of the Jacobian matrix hPrime to 1.0
  hPrime(0, 0) = 1.0; // Set element (0,0) to 1.0
  hPrime(1, 1) = 1.0; // Set element (1,1) to 1.0
  hPrime(2, 2) = 1.0; // Set element (2,2) to 1.0
  hPrime(3, 3) = 1.0; // Set element (3,3) to 1.0
  hPrime(4, 4) = 1.0; // Set element (4,4) to 1.0
  hPrime(5, 5) = 1.0; // Set element (5,5) to 1.0

  // Setting current estimated position from the EKF state
  zFromX(0) = ekfState(0); // Position x
  zFromX(1) = ekfState(1); // Position y
  zFromX(2) = ekfState(2); // Position z
  zFromX(3) = ekfState(3); // Velocity x
  zFromX(4) = ekfState(4); // Velocity y
  zFromX(5) = ekfState(5); // Velocity z

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  Update(z, hPrime, R_GPS, zFromX);
}

void QuadEstimatorEKF::UpdateFromMag(float magYaw)
{
  VectorXf z(1), zFromX(1);
  z(0) = magYaw;

  MatrixXf hPrime(1, QUAD_EKF_NUM_STATES);
  hPrime.setZero();

  // MAGNETOMETER UPDATE
  // Hints:
  //  - Your current estimated yaw can be found in the state vector: ekfState(6)
  //  - Make sure to normalize the difference between your measured and estimated yaw
  //    (you don't want to update your yaw the long way around the circle)
  //  - The magnetomer measurement covariance is available in member variable R_Mag
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Set the missing value of the Jacobian matrix hPrime to 1.0 for the yaw component
  hPrime(0, 6) = 1.0; // Jacobian entry related to yaw measurement

  // Assign the current estimated yaw from the EKF state to the measurement vector
  zFromX(0) = ekfState(6); // Current estimated yaw angle

  // Calculate the difference between measured and estimated yaw
  VectorXf yawDifference = z - zFromX; // Difference between measurements

  // Normalize the yaw difference to ensure it lies within the range [-π, π]
  if (yawDifference(0) > F_PI)
    z(0) -= 2.f * F_PI; // Adjust measured yaw if greater than π
  if (yawDifference(0) < -F_PI)
    z(0) += 2.f * F_PI; // Adjust measured yaw if less than -π

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  Update(z, hPrime, R_Mag, zFromX);
}

// Execute an EKF update step
// z: measurement
// H: Jacobian of observation function evaluated at the current estimated state
// R: observation error model covariance
// zFromX: measurement prediction based on current state
void QuadEstimatorEKF::Update(VectorXf &z, MatrixXf &H, MatrixXf &R, VectorXf &zFromX)
{
  assert(z.size() == H.rows());
  assert(QUAD_EKF_NUM_STATES == H.cols());
  assert(z.size() == R.rows());
  assert(z.size() == R.cols());
  assert(z.size() == zFromX.size());

  MatrixXf toInvert(z.size(), z.size());
  toInvert = H * ekfCov * H.transpose() + R;
  MatrixXf K = ekfCov * H.transpose() * toInvert.inverse();

  ekfState = ekfState + K * (z - zFromX);

  MatrixXf eye(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES);
  eye.setIdentity();

  ekfCov = (eye - K * H) * ekfCov;
}

// Calculate the condition number of the EKF ovariance matrix (useful for numerical diagnostics)
// The condition number provides a measure of how similar the magnitudes of the error metric beliefs
// about the different states are. If the magnitudes are very far apart, numerical issues will start to come up.
float QuadEstimatorEKF::CovConditionNumber() const
{
  MatrixXf m(7, 7);
  for (int i = 0; i < 7; i++)
  {
    for (int j = 0; j < 7; j++)
    {
      m(i, j) = ekfCov(i, j);
    }
  }

  Eigen::JacobiSVD<MatrixXf> svd(m);
  float cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
  return cond;
}

// Access functions for graphing variables
bool QuadEstimatorEKF::GetData(const string &name, float &ret) const
{
  if (name.find_first_of(".") == string::npos)
    return false;
  string leftPart = LeftOf(name, '.');
  string rightPart = RightOf(name, '.');

  if (ToUpper(leftPart) == ToUpper(_name))
  {
#define GETTER_HELPER(A, B)                       \
  if (SLR::ToUpper(rightPart) == SLR::ToUpper(A)) \
  {                                               \
    ret = (B);                                    \
    return true;                                  \
  }
    GETTER_HELPER("Est.roll", rollEst);
    GETTER_HELPER("Est.pitch", pitchEst);

    GETTER_HELPER("Est.x", ekfState(0));
    GETTER_HELPER("Est.y", ekfState(1));
    GETTER_HELPER("Est.z", ekfState(2));
    GETTER_HELPER("Est.vx", ekfState(3));
    GETTER_HELPER("Est.vy", ekfState(4));
    GETTER_HELPER("Est.vz", ekfState(5));
    GETTER_HELPER("Est.yaw", ekfState(6));

    GETTER_HELPER("Est.S.x", sqrtf(ekfCov(0, 0)));
    GETTER_HELPER("Est.S.y", sqrtf(ekfCov(1, 1)));
    GETTER_HELPER("Est.S.z", sqrtf(ekfCov(2, 2)));
    GETTER_HELPER("Est.S.vx", sqrtf(ekfCov(3, 3)));
    GETTER_HELPER("Est.S.vy", sqrtf(ekfCov(4, 4)));
    GETTER_HELPER("Est.S.vz", sqrtf(ekfCov(5, 5)));
    GETTER_HELPER("Est.S.yaw", sqrtf(ekfCov(6, 6)));

    // diagnostic variables
    GETTER_HELPER("Est.D.AccelPitch", accelPitch);
    GETTER_HELPER("Est.D.AccelRoll", accelRoll);

    GETTER_HELPER("Est.D.ax_g", accelG[0]);
    GETTER_HELPER("Est.D.ay_g", accelG[1]);
    GETTER_HELPER("Est.D.az_g", accelG[2]);

    GETTER_HELPER("Est.E.x", trueError(0));
    GETTER_HELPER("Est.E.y", trueError(1));
    GETTER_HELPER("Est.E.z", trueError(2));
    GETTER_HELPER("Est.E.vx", trueError(3));
    GETTER_HELPER("Est.E.vy", trueError(4));
    GETTER_HELPER("Est.E.vz", trueError(5));
    GETTER_HELPER("Est.E.yaw", trueError(6));
    GETTER_HELPER("Est.E.pitch", pitchErr);
    GETTER_HELPER("Est.E.roll", rollErr);
    GETTER_HELPER("Est.E.MaxEuler", maxEuler);

    GETTER_HELPER("Est.E.pos", posErrorMag);
    GETTER_HELPER("Est.E.vel", velErrorMag);

    GETTER_HELPER("Est.D.covCond", CovConditionNumber());
#undef GETTER_HELPER
  }
  return false;
};

vector<string> QuadEstimatorEKF::GetFields() const
{
  vector<string> ret = BaseQuadEstimator::GetFields();
  ret.push_back(_name + ".Est.roll");
  ret.push_back(_name + ".Est.pitch");

  ret.push_back(_name + ".Est.x");
  ret.push_back(_name + ".Est.y");
  ret.push_back(_name + ".Est.z");
  ret.push_back(_name + ".Est.vx");
  ret.push_back(_name + ".Est.vy");
  ret.push_back(_name + ".Est.vz");
  ret.push_back(_name + ".Est.yaw");

  ret.push_back(_name + ".Est.S.x");
  ret.push_back(_name + ".Est.S.y");
  ret.push_back(_name + ".Est.S.z");
  ret.push_back(_name + ".Est.S.vx");
  ret.push_back(_name + ".Est.S.vy");
  ret.push_back(_name + ".Est.S.vz");
  ret.push_back(_name + ".Est.S.yaw");

  ret.push_back(_name + ".Est.E.x");
  ret.push_back(_name + ".Est.E.y");
  ret.push_back(_name + ".Est.E.z");
  ret.push_back(_name + ".Est.E.vx");
  ret.push_back(_name + ".Est.E.vy");
  ret.push_back(_name + ".Est.E.vz");
  ret.push_back(_name + ".Est.E.yaw");
  ret.push_back(_name + ".Est.E.pitch");
  ret.push_back(_name + ".Est.E.roll");

  ret.push_back(_name + ".Est.E.pos");
  ret.push_back(_name + ".Est.E.vel");

  ret.push_back(_name + ".Est.E.maxEuler");

  ret.push_back(_name + ".Est.D.covCond");

  // diagnostic variables
  ret.push_back(_name + ".Est.D.AccelPitch");
  ret.push_back(_name + ".Est.D.AccelRoll");
  ret.push_back(_name + ".Est.D.ax_g");
  ret.push_back(_name + ".Est.D.ay_g");
  ret.push_back(_name + ".Est.D.az_g");
  return ret;
};
