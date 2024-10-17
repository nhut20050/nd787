#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float l = L / sqrtf(2.f);  // Calculate the lever arm length for diagonal motors based on the drone configuration
  
  float S = collThrustCmd / 4.f;  // Compute the average thrust contribution for each motor
  
  // Compute the contributions of moments along the roll, pitch, and yaw axes
  float X = momentCmd.x / (l * 4.f);  // Roll contribution to motor thrust
  float Y = momentCmd.y / (l * 4.f);  // Pitch contribution to motor thrust
  float Z = momentCmd.z / (kappa * 4.f);  // Yaw contribution to motor thrust
  
  // Calculate the individual motor thrusts by combining thrust and moment contributions
  cmd.desiredThrustsN[0] = S + X + Y - Z;  // Front-left motor
  cmd.desiredThrustsN[1] = S - X + Y + Z;  // Front-right motor
  cmd.desiredThrustsN[2] = S + X - Y + Z;  // Rear-left motor
  cmd.desiredThrustsN[3] = S - X - Y - Z;  // Rear-right motor

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Calculate desired angular accelerations using proportional control on body rate errors
  V3F pqr_error = pqrCmd - pqr;  // Compute the error between desired and actual angular velocities

  momentCmd.x = Ixx * kpPQR.x * pqr_error.x;  // Roll moment
  momentCmd.y = Iyy * kpPQR.y * pqr_error.y;  // Pitch moment
  momentCmd.z = Izz * kpPQR.z * pqr_error.z;  // Yaw moment

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Convert thrust to acceleration and constrain desired roll and pitch accelerations
  float c = -collThrustCmd / mass;  // Downward acceleration from collective thrust
  
  float bx = CONSTRAIN(accelCmd.x / c, -sin(maxTiltAngle), sin(maxTiltAngle));  // Desired roll
  float by = CONSTRAIN(accelCmd.y / c, -sin(maxTiltAngle), sin(maxTiltAngle));  // Desired pitch
  
  // Compute control effort for roll and pitch using proportional control
  float bx_dot = kpBank * (bx - R(0, 2));  // Roll control effort
  float by_dot = kpBank * (by - R(1, 2));  // Pitch control effort
  
  // Convert control efforts to desired body frame roll and pitch rates
  pqrCmd.x = (R(1, 0) * bx_dot - R(0, 0) * by_dot) / R(2, 2);  // Roll rate command
  pqrCmd.y = (R(1, 1) * bx_dot - R(0, 1) * by_dot) / R(2, 2);  // Pitch rate command

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Compute the altitude error and update the integral for PID control
  float pos_err = posZCmd - posZ;  // Altitude error
  integratedAltitudeError += pos_err * dt;  // Update integral error
  
  // Calculate the desired velocity and clamp it within max ascent/descent rates
  float vel_cmd = kpPosZ * pos_err + velZCmd;  // Desired velocity
  vel_cmd = CONSTRAIN(vel_cmd, -maxAscentRate, maxDescentRate);  // Clamp velocity
  
  // Compute the velocity error and use it to calculate the required acceleration
  float vel_err = vel_cmd - velZ;  // Velocity error
  float acc_cmd = kpVelZ * vel_err + accelZCmd + KiPosZ * integratedAltitudeError;  // Desired acceleration
  
  // Convert acceleration to thrust and constrain it within min/max motor thrust limits
  thrust = mass * (CONST_GRAVITY - acc_cmd) / R(2, 2);  // Required thrust
  thrust = CONSTRAIN(thrust, 4 * minMotorThrust, 4 * maxMotorThrust);  // Clamp thrust

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Compute desired velocity in XY plane and constrain it within the max speed
  V3F vel_cmd = kpPosXY * (posCmd - pos) + velCmd;  // Desired velocity
  if (vel_cmd.mag() > maxSpeedXY) {
	vel_cmd = vel_cmd.norm() * maxSpeedXY;  // Clamp velocity magnitude
  }
  
  // Calculate the acceleration command from the velocity error
  accelCmd += kpVelXY * (vel_cmd - vel);  // Desired acceleration
  accelCmd.z = 0;  // Ensure no vertical component
  
  // Constrain the horizontal acceleration within the max acceleration limit
  if (accelCmd.mag() > maxAccelXY) {
	accelCmd = accelCmd.norm() * maxAccelXY;  // Clamp acceleration
  }

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Wrap the yaw command within [-pi, pi] to prevent discontinuities
  if (yawCmd > 0) {
	yawCmd = fmodf(yawCmd, 2 * F_PI);  // Wrap positive yaw command
  } else {
	yawCmd = -fmodf(-yawCmd, 2 * F_PI);  // Wrap negative yaw command
  }
  
  // Calculate the yaw error and wrap it within [-pi, pi]
  float err = yawCmd - yaw;  // Yaw error
  if (err > F_PI) {
	err -= 2 * F_PI;  // Adjust for overflow
  } else if (err < -F_PI) {
	err += 2 * F_PI;  // Adjust for underflow
  }
  
  // Compute the desired yaw rate using proportional control
  yawRateCmd = kpYaw * err;  // Yaw rate command

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
