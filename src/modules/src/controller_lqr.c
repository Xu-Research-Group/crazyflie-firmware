/**
 *   Authored by Victor Freire (https://xu.me.wisc.edu/), May 2021
 *
 *
 */

#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "pid.h"
#include "sensfusion6.h"
#include "controller_lqr.h"

#include "log.h"
#include "param.h"
#include "math3d.h"
#include "cf_math.h"
#include "debug.h"

#include "crtp_commander_sdlqr.h"

#ifdef CF_CBF
#include "osqp.h"
#include "workspace.h"
#endif

#ifdef AI_CBF
#include "aideck.h"
#endif

#ifndef CF_MASS
#define CF_MASS            0.032f
#endif

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

#ifdef LQR_ALT_PID
#define Z_PID_UPDATE_DT    (float)(1.0f/Z_PID_RATE)
#define PID_Z_KI           1.0f
#define Z_LPF_CUTOFF_FREQ  20.0f
#endif

#define DEG2RAD               (float)M_PI/180.0f
#define RAD2DEG               180.0f/(float)M_PI


// The mode is a parameter that can be changed from the client-side
static lqr_mode_t mode = D9LQR; // Default mode

static state_t err;
static attitude_t rateDesired;
static float u[4]; // Control input u = [T p q r]
static float u_D6[4]; // Control input u = [T phi theta psi]
static float actuatorThrust;
static float KD9[4][9]; // Kalman Gain (9D)
static float KD6[4][6]; // Kalman Gain (6D)
#ifdef LQR_ALT_PID
PidObject pidT; // PID object for altitude (integral)
#endif
static bool flying = false; // Set thrust to 0 when false

#ifdef AI_CBF
static cbf_qp_data_t cbf_qp_data;
#endif

// Logging parameters
#ifdef LQR_ALT_PID
static float pid_T;
#endif
static float u_T;
static float u_p;
static float u_q;
static float u_r;

// Private function: Takes normalized thrust (m/s^2) and returns pwm units
static int to_pwm(float T){
  // 0 = a*(rpm)^2 -b*rpm + c - mass_in_gram
  float a = 109e-9f;
  float b = 210.6e-6f;
  float c = 0.154f;
  // RPM -> PWM conversion
  // rpm = d*pwm + e
  float d = 0.2685f;
  float e = 4070.3f;
  // Convert T to grams
  float g = (CF_MASS*1000.0f*T)/9.81f; // Mass in grams

  float r = (b+sqrtf(powf(b,2)-4*a*(c-g)))/(2*a); // Quadratic formula (+)
  int pwm = (int) ((r-e)/d);
  pwm -= 9000; // Offset calibration
  return pwm;
}

// Private function: D9LQR Policy update
static void lqr_D9(setpoint_t *setpoint, const state_t *state, const uint32_t tick){
  if (RATE_DO_EXECUTE(D9LQR_RATE, tick)) {
    // Compute errors in state err = \hat{x} - x_r
    err.position.x = state->position.x - setpoint->position.x;
    err.position.y = state->position.y - setpoint->position.y;
    err.position.z = state->position.z - setpoint->position.z;
    err.attitude.roll = state->attitude.roll*DEG2RAD - setpoint->attitude.roll;
    err.attitude.pitch = -state->attitude.pitch*DEG2RAD - setpoint->attitude.pitch;
    err.attitude.yaw = state->attitude.yaw*DEG2RAD - setpoint->attitude.yaw;
    err.velocity.x = state->velocity.x - setpoint->velocity.x;
    err.velocity.y = state->velocity.y - setpoint->velocity.y;
    err.velocity.z = state->velocity.z - setpoint->velocity.z;

    // Negative state feedback: \deltau = -K*err
    u[0] = -(KD9[0][0]*err.position.x    + KD9[0][1]*err.position.y     + KD9[0][2]*err.position.z
           + KD9[0][3]*err.attitude.roll + KD9[0][4]*err.attitude.pitch + KD9[0][5]*err.attitude.yaw
           + KD9[0][6]*err.velocity.x    + KD9[0][7]*err.velocity.y     + KD9[0][8]*err.velocity.z);
    u[1] = -(KD9[1][0]*err.position.x    + KD9[1][1]*err.position.y     + KD9[1][2]*err.position.z
           + KD9[1][3]*err.attitude.roll + KD9[1][4]*err.attitude.pitch + KD9[1][5]*err.attitude.yaw
           + KD9[1][6]*err.velocity.x    + KD9[1][7]*err.velocity.y     + KD9[1][8]*err.velocity.z);
    u[2] = -(KD9[2][0]*err.position.x    + KD9[2][1]*err.position.y     + KD9[2][2]*err.position.z
           + KD9[2][3]*err.attitude.roll + KD9[2][4]*err.attitude.pitch + KD9[2][5]*err.attitude.yaw
           + KD9[2][6]*err.velocity.x    + KD9[2][7]*err.velocity.y     + KD9[2][8]*err.velocity.z);
    u[3] = -(KD9[3][0]*err.position.x    + KD9[3][1]*err.position.y     + KD9[3][2]*err.position.z
           + KD9[3][3]*err.attitude.roll + KD9[3][4]*err.attitude.pitch + KD9[3][5]*err.attitude.yaw
           + KD9[3][6]*err.velocity.x    + KD9[3][7]*err.velocity.y     + KD9[3][8]*err.velocity.z);

    // Add nominal control:  u = u_r + \deltau
    u[0] += setpoint->thrust;
    u[1] += setpoint->attitudeRate.roll;
    u[2] += setpoint->attitudeRate.pitch;
    u[3] += setpoint->attitudeRate.yaw;
  } // if RATE_DO_EXECUTE

} // lqr_D9()

// Private function: D6LQR Policy update
static void lqr_D6(setpoint_t *setpoint, const state_t *state, const uint32_t tick){
  if (RATE_DO_EXECUTE(D6LQR_RATE, tick)) {
    // Compute errors in state err = \hat{x} - x_r
    err.position.x = state->position.x - setpoint->position.x;
    err.position.y = state->position.y - setpoint->position.y;
    err.position.z = state->position.z - setpoint->position.z;
    err.velocity.x = state->velocity.x - setpoint->velocity.x;
    err.velocity.y = state->velocity.y - setpoint->velocity.y;
    err.velocity.z = state->velocity.z - setpoint->velocity.z;

    // Negative state feedback: \deltau = -K*err
    u_D6[0] = -(KD6[0][0]*err.position.x   + KD6[0][1]*err.position.y   + KD6[0][2]*err.position.z
              + KD6[0][3]*err.velocity.x   + KD6[0][4]*err.velocity.y   + KD6[0][5]*err.velocity.z);
    u_D6[1] = -(KD6[1][0]*err.position.x   + KD6[1][1]*err.position.y   + KD6[1][2]*err.position.z
              + KD6[1][3]*err.velocity.x   + KD6[1][4]*err.velocity.y   + KD6[1][5]*err.velocity.z);
    u_D6[2] = -(KD6[2][0]*err.position.x   + KD6[2][1]*err.position.y   + KD6[2][2]*err.position.z
              + KD6[2][3]*err.velocity.x   + KD6[2][4]*err.velocity.y   + KD6[2][5]*err.velocity.z);
    u_D6[3] = -(KD6[3][0]*err.position.x   + KD6[3][1]*err.position.y   + KD6[3][2]*err.position.z
              + KD6[3][3]*err.velocity.x   + KD6[3][4]*err.velocity.y   + KD6[3][5]*err.velocity.z);

    // Add nominal control:  u = u_r + \deltau
    u_D6[0] += setpoint->thrust;
    u_D6[1] += setpoint->attitude.roll;
    u_D6[2] += setpoint->attitude.pitch;
    u_D6[3] += setpoint->attitude.yaw;
  } // if RATE_DO_EXECUTE

} // lqr_D6()

#ifdef CF_CBF
/**
 * Private function. Solve a QP-CBF to limit position [x y]
 * The osqp workspace is generated before compilation, see:
 * ~/tools/make/osqp/osqp_code_gen.py
 */
static int apply_cbf(const state_t *state){
  // Get vars
  float x_ddot;
  float y_ddot;
  float z_ddot;
  float T = u_D6[0];
  float phi = u_D6[1];
  float theta = u_D6[2];
  float psi = u_D6[3];
  float x = state->position.x;
  float y = state->position.y;
  float x_dot = state->velocity.x;
  float y_dot = state->velocity.y;

  // Virtualize inputs
  c_float mu_nom[3];
  mu_nom[0] = T*(arm_sin_f32(phi)*arm_sin_f32(psi) +
            arm_cos_f32(phi)*arm_sin_f32(theta)*arm_cos_f32(psi)); // x_ddot
  mu_nom[1] = T*(arm_cos_f32(phi)*arm_sin_f32(theta)*arm_sin_f32(psi) -
                 arm_sin_f32(phi)*arm_cos_f32(psi)); // y_ddot
  mu_nom[2] = T*arm_cos_f32(phi)*arm_cos_f32(theta) - 9.81f; // z_ddot

  // Update Linear Cost  q
  c_float q[3] = {-2.0*mu_nom[0], -2.0*mu_nom[1], -2.0*mu_nom[2]};
  osqp_update_lin_cost(&workspace,q); // Update q

  // Update Lower Bound
  c_float b_lower[4];
  b_lower[0] = -(CF_CBF_K1*(CF_CBF_X_MAX-x) + CF_CBF_K2*(-x_dot));
  b_lower[1] = -(CF_CBF_K1*(x-CF_CBF_X_MIN) + CF_CBF_K2*x_dot);
  b_lower[2] = -(CF_CBF_K1*(CF_CBF_Y_MAX-y) + CF_CBF_K2*(-y_dot));
  b_lower[3] = -(CF_CBF_K1*(y-CF_CBF_Y_MIN) + CF_CBF_K2*y_dot);
  osqp_update_lower_bound(&workspace,b_lower);

#ifdef CF_CBF_MOCK
  // Solve mock CBF-QP
  int exitflag = 0; // Success
  x_ddot = mu_nom[0];
  y_ddot = mu_nom[1];
  z_ddot = mu_nom[2];
#else
  // Solve CBF-QP
  int exitflag = osqp_solve(&workspace);
  if(exitflag){ // Errors
    DEBUG_PRINT("OSQP Solve error %d\n",exitflag);
    x_ddot = mu_nom[0];
    y_ddot = mu_nom[1];
    z_ddot = mu_nom[2];
  }
  else{
    x_ddot = workspace.solution->x[0];
    y_ddot = workspace.solution->x[1];
    z_ddot = workspace.solution->x[2];
  }
#endif

  // Recover inputs
  u_D6[3] = psi; // psi
  u_D6[2] = atan((arm_cos_f32(psi)*x_ddot + arm_sin_f32(psi)*y_ddot)/(z_ddot+9.81f)); // theta
  u_D6[1] = atan((arm_sin_f32(psi)*x_ddot - arm_cos_f32(psi)*y_ddot)/(z_ddot+9.81f)*arm_cos_f32(u_D6[2])); // phi
  u_D6[0] = (z_ddot+9.81f)/(arm_cos_f32(u_D6[1])*arm_cos_f32(u_D6[2])); // T

  // Return
  return exitflag;
}
#endif // #ifdef CF_CBF



void controllerLqrInit(void){
  // Initialize 9-Dim Kalman gain with default Linearization
  KD9[0][2] = 31.6228f;
  KD9[0][8] = 12.7768f;
  KD9[1][1] = -4.4732f;
  KD9[1][3] = 7.6869f;
  KD9[1][7] = -3.0014f;
  KD9[2][0] = 4.4732f;
  KD9[2][4] = 7.6869f;
  KD9[2][6] = 3.0014f;
  KD9[3][5] = 1.0f;

  // Initialize 6-Dim Kalman gain with default Linearization
  KD6[0][2] = 31.6228f;
  KD6[0][5] =  8.5584f;
  KD6[1][1] = -2.2361f;
  KD6[1][4] = -0.7112f;
  KD6[2][0] =  2.2361f;
  KD6[2][3] =  0.7112f;

#ifdef LQR_ALT_PID
  // Initialize altitude pid (T)
  pidInit(&pidT, 0, 0, PID_Z_KI, 0, Z_PID_UPDATE_DT, Z_PID_RATE,
           Z_LPF_CUTOFF_FREQ, false);
  pidSetIntegralLimit(&pidT, 0.5); // [m] integral limit
  pidT.outputLimit = 0.5; // [m/s^2] output limit
#endif

  // Initialize attitude controller
  attitudeControllerInit(ATTITUDE_UPDATE_DT);

}

bool controllerLqrTest(void)
{
  bool pass = true;

#ifdef CF_CBF
  pass &= OSQP_ENABLED; // OSQP Must be compiled to use the CF_CBF
#endif

  pass &= attitudeControllerTest();

  return pass;
}


void controllerLqr(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors,
                   const state_t *state, const uint32_t tick){
  // Flying safety
  if (setpoint->position.z > 0)
    flying = true;
  else
    flying = false;

  // Perform the LQR Update
  if (mode==D9LQR)
    lqr_D9(setpoint, state, tick); // update u
  else if (mode==D6LQR){
    lqr_D6(setpoint, state, tick); // update u_D6
  }

  // Apply CF_CBF
#ifdef CF_CBF
  if (mode==D6LQR){
    if (RATE_DO_EXECUTE(D6LQR_RATE, tick))
      apply_cbf(state);
  }
#endif


  // Perform the Attitude PID Update if needed
  if (mode==D6LQR){ // update u
    u[0] = u_D6[0]; // T is the same
    if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)){
      attitudeControllerCorrectAttitudePID(state->attitude.roll, -state->attitude.pitch, state->attitude.yaw,
                               u_D6[1]*RAD2DEG, u_D6[2]*RAD2DEG, u_D6[3]*RAD2DEG,
                               &(u[1]), &(u[2]), &(u[3]) );
      // Convert to rad/s
      u[1] = u[1]*DEG2RAD;
      u[2] = u[2]*DEG2RAD;
      u[3] = u[3]*DEG2RAD;
    }
  } // If mode==D6LQR

#ifdef LQR_ALT_PID
  // Add altitude integral action
  if(RATE_DO_EXECUTE(Z_PID_RATE, tick)){
    pidSetDesired(&pidT, setpoint->position.z);
    pid_T = pidUpdate(&pidT, state->position.z, true);
    u[0] += pid_T;
  }
#endif

  // Apply AI_CBF if needed
#ifdef AI_CBF
  // Populate cbf_qp_data
  cbf_qp_data.phi = state->attitude.roll*DEG2RAD;
  cbf_qp_data.theta = -state->attitude.pitch*DEG2RAD;
  cbf_qp_data.u.T = u[0];
  cbf_qp_data.u.p = u[1];
  cbf_qp_data.u.q = u[2];
  cbf_qp_data.u.r = u[3];
  // Send to AI Deck
  aideck_send_cbf_data(&cbf_qp_data);
  // Get the most recent safe control received from AI Deck
  aideck_get_safe_u(u);
#endif

  // Saturate thrust and pqr
  u[0] = fmaxf(fminf(u[0],18.0f), 0.0f); // [0,18] m/s^2
  u[1] = fmaxf(fminf(u[1],3.5f),-3.5f);  // +-200 deg/s
  u[2] = fmaxf(fminf(u[2],3.5f),-3.5f);  // +-200 deg/s
  u[3] = fmaxf(fminf(u[3],3.5f),-3.5f);  // +-200 deg/s

  // Parameter logging
  u_T = u[0];
  u_p = u[1];
  u_q = u[2];
  u_r = u[3];

  // convert [T, p, q, r] to 0xFFFF and deg/s
  actuatorThrust = to_pwm(u[0]);
  rateDesired.roll = u[1]*RAD2DEG;
  rateDesired.pitch = u[2]*RAD2DEG;
  rateDesired.yaw = u[3]*RAD2DEG;

  // Set thrust to 0 if z_desired is 0 and we are close to target
  if((err.position.x + err.position.y + err.position.y) < 0.075f && setpoint->position.z == 0)
      flying = false;
  if(!flying){
      actuatorThrust = 0;
  }

  // Attitude Rate Controller 500 Hz
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Attitude Rate PID update
    attitudeControllerCorrectRatePID(sensors->gyro.x, sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);
    // Power Distribution variables update
    attitudeControllerGetActuatorOutput(&control->roll, &control->pitch, &control->yaw);
  }

  // Power Distribution variable update
  control->thrust = actuatorThrust;

  // Safety for power distribution
  if (control->thrust == 0)
  {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    attitudeControllerResetAllPID();
#ifdef LQR_ALT_PID
    pidReset(&pidT); // Reset the altitude pid
#endif
  }
}

// Public function to update entries of KD9 (see crtp_commander_sdlqr)
void update_K_entry(const uint8_t i, const uint8_t j, float value){
    KD9[i][j] = value;
}


/**
 * Parameters to set the LQR Controller mode (D6LQR or D9LQR)
 */
PARAM_GROUP_START(controller_lqr)
/**
 * @brief LQR Controller mode D9LQR(0), D6LQR(1), (Default: 0)
 */
PARAM_ADD_CORE(PARAM_UINT8, mode, &mode)
PARAM_GROUP_STOP(controller_lqr)

/**
 * Logging variables
 */
LOG_GROUP_START(controller_lqr)
LOG_ADD(LOG_FLOAT, u_T, &u_T)
LOG_ADD(LOG_FLOAT, u_p, &u_p)
LOG_ADD(LOG_FLOAT, u_q, &u_q)
LOG_ADD(LOG_FLOAT, u_r, &u_r)
#ifdef LQR_ALT_PID
LOG_ADD(LOG_FLOAT, pid_T, &pid_T)
#endif
LOG_GROUP_STOP(controller_lqr)
