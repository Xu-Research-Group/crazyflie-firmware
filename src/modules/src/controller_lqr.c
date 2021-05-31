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

#ifdef OSQP_ENABLED
#include "osqp.h"
#include "workspace.h"
#endif

#ifdef AI_CBF
#include "deck.h"
u_t aideckRxBuffer;
volatile uint8_t dma_flag = 0;
#endif

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)
#define Z_PID_UPDATE_DT    (float)(1.0f/POSITION_RATE)
#define PID_Z_KI           1.0f
#define Z_LPF_CUTOFF_FREQ  20.0f
#define DEG2RAD               (float)M_PI/180.0f
#define RAD2DEG               180.0f/(float)M_PI

static bool tiltCompensationEnabled = false;

static attitude_t rateDesired;
static state_t err;
static float actuatorThrust;
static float K[4][9]; // Kalman Gain
PidObject pidT; // PID object for altitude (integral)

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static bool flying = false;

// Logging parameters
static float u_T;
static float u_p;
static float u_q;
static float u_r;
static float pid_T;

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

void controllerLqrInit(void)
{
  // Initialize Kalman gain with default Linearization
  K[0][2] = 31.6228f;
  K[0][8] = 12.7768f;
  K[1][1] = -4.4732f;
  K[1][3] = 7.6869f;
  K[1][7] = -3.0014f;
  K[2][0] = 4.4732f;
  K[2][4] = 7.6869f;
  K[2][6] = 3.0014f;
  K[3][5] = 1.0f;

  // Initialize altitude pid (T)
  pidInit(&pidT, 0, 0, PID_Z_KI, 0, Z_PID_UPDATE_DT, POSITION_RATE,
           Z_LPF_CUTOFF_FREQ, false);
  pidSetIntegralLimit(&pidT, 0.5); // [m] integral limit
  pidT.outputLimit = 0.5; // [m/s^2] output limit

  // Initialize attitude rate controller
  attitudeControllerInit(ATTITUDE_UPDATE_DT);

  #ifdef AI_CBF
  // Initialize DMA for AI Deck Receiving of u
  USART_DMA_Start(115200, &aideckRxBuffer, sizeof(u_t));
  #endif

}

bool controllerLqrTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}


void controllerLqr(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
    //Run LQR and update [c, p, q, r]
    if (setpoint->position.z > 0){
        flying = true;
    }
    else
        flying = false;

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
    actuatorThrust    = -(K[0][0]*err.position.x    + K[0][1]*err.position.y     + K[0][2]*err.position.z
                        + K[0][3]*err.attitude.roll + K[0][4]*err.attitude.pitch + K[0][5]*err.attitude.yaw
                        + K[0][6]*err.velocity.x    + K[0][7]*err.velocity.y     + K[0][8]*err.velocity.z);   // T (norm thrust)

    rateDesired.roll  = -(K[1][0]*err.position.x    + K[1][1]*err.position.y     + K[1][2]*err.position.z
                        + K[1][3]*err.attitude.roll + K[1][4]*err.attitude.pitch + K[1][5]*err.attitude.yaw
                        + K[1][6]*err.velocity.x    + K[1][7]*err.velocity.y     + K[1][8]*err.velocity.z);   // p

    rateDesired.pitch = -(K[2][0]*err.position.x    + K[2][1]*err.position.y     + K[2][2]*err.position.z
                        + K[2][3]*err.attitude.roll + K[2][4]*err.attitude.pitch + K[2][5]*err.attitude.yaw
                        + K[2][6]*err.velocity.x    + K[2][7]*err.velocity.y     + K[2][8]*err.velocity.z);   // q

    rateDesired.yaw   = -(K[3][0]*err.position.x    + K[3][1]*err.position.y     + K[3][2]*err.position.z
                        + K[3][3]*err.attitude.roll + K[3][4]*err.attitude.pitch + K[3][5]*err.attitude.yaw
                        + K[3][6]*err.velocity.x    + K[3][7]*err.velocity.y     + K[3][8]*err.velocity.z);   // r


    // Add nominal control:  u = u_r + \deltau
    actuatorThrust += setpoint->thrust;
    rateDesired.roll += setpoint->attitudeRate.roll;
    rateDesired.pitch += setpoint->attitudeRate.pitch;
    rateDesired.yaw += setpoint->attitudeRate.yaw;

    // Add altitude integral action
    pidSetDesired(&pidT, setpoint->position.z);
    pid_T = pidUpdate(&pidT, state->position.z, true);
    actuatorThrust += pid_T;

    // Parameter logging
    u_T = actuatorThrust;
    u_p = rateDesired.roll;
    u_q = rateDesired.pitch;
    u_r = rateDesired.yaw;

    // Apply CBF if enabled TODO: add CBF flag besides OSQP flag
    #ifdef OSQP_ENABLED
	if(flying){
      apply_cbf(state,10.0f, (float)EPSILON_CBF*DEG2RAD);
    }
    #endif

    #ifdef AI_CBF
    // Apply AI Deck CBF-QP TODO
    if(dma_flag==1){
        dma_flag = 0; // Clear the flag
        DEBUG_PRINT("CBFQP: u[0] = %.4f\n",(double)aideckRxBuffer.T);
        memset(&aideckRxBuffer, 0, sizeof(u_t));
    }
    #endif

    // Saturate thrust and pqr
    actuatorThrust = fmaxf(fminf(actuatorThrust,18), 2);
    rateDesired.roll = fmaxf(fminf(rateDesired.roll,6),-6);
    rateDesired.pitch = fmaxf(fminf(rateDesired.pitch,6),-6);
    rateDesired.yaw = fmaxf(fminf(rateDesired.yaw,6),-6);

    // convert [T, p, q, r] to 0xFFFF and deg/s
    actuatorThrust = to_pwm(actuatorThrust);
    //actuatorThrust = 19549.0f*sqrtf(actuatorThrust) - 15159.0f; // Convert to PWM units
    rateDesired.roll = rateDesired.roll*RAD2DEG;
    rateDesired.pitch = rateDesired.pitch*RAD2DEG;
    rateDesired.yaw = rateDesired.yaw*RAD2DEG;

    // Set thrust to 0 if z_desired is 0 and we are close to target
    if((err.position.x + err.position.y + err.position.y) < 0.075f && setpoint->position.z == 0)
        flying = false;
    if(!flying){
        actuatorThrust = 0;
    }

  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // TODO: Investigate possibility to subtract gyro drift.
    attitudeControllerCorrectRatePID(sensors->gyro.x, sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = -radians(sensors->gyro.z);
  }

  if (tiltCompensationEnabled)
  {
    control->thrust = actuatorThrust / sensfusion6GetInvThrustCompensationForTilt();
  }
  else
  {
    control->thrust = actuatorThrust;
  }

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();
    pidReset(&pidT); // Reset the altitude pid

  }
}

// Public function to update entries of K (see crtp_commander_sdlqr)
void update_K_entry(const uint8_t i, const uint8_t j, float value){
    K[i][j] = value;
}

#ifdef OSQP_ENABLED
// Use OSQP library to solve a QP-CBF
// The osqp workspace is generated before compilation, see:
// ~/tools/make/osqp/osqp_code_gen.py
int apply_cbf(const state_t *state, const float k, const float epsilon){
  // Convenient temp variables
  float phi = state->attitude.roll*DEG2RAD;
  float theta = -state->attitude.pitch*DEG2RAD; // Opposite sign pitch from estimator

  // Update Linear Cost: -2*u_LQR
  c_float q[4] = {-2.0f*actuatorThrust, -2.0f*rateDesired.roll,
                  -2.0f*rateDesired.pitch, -2.0f*rateDesired.yaw, }; // -2*u_LQR
  osqp_update_lin_cost(&workspace, q); // Update q

  // Update Constraint matrix A (CBF)
  float s_phi = arm_sin_f32((float)M_PI*phi/(2.0f*epsilon));
  float s_theta = arm_sin_f32((float)M_PI*theta/(2.0f*epsilon));
  c_float A_x[5] = {s_phi,
                    s_phi*arm_sin_f32(phi)*arm_sin_f32(theta)/arm_cos_f32(theta),
                    s_theta*arm_cos_f32(phi),
                    s_phi*arm_cos_f32(phi)*arm_sin_f32(theta)/arm_cos_f32(theta),
                      -s_theta*arm_sin_f32(phi), };
  osqp_update_A(&workspace, A_x, OSQP_NULL, 5);

  // Update constraint upper bound (CBF)
  // Lower bound is set to -9999 (-infty desired)
  c_float u[2] = {k*arm_cos_f32((float)M_PI*state->attitude.roll/(2.0f*epsilon)),
                  k*arm_cos_f32((float)M_PI*-state->attitude.pitch/(2.0f*epsilon)), }; // Upper bound NOTE pitch is opposite
  osqp_update_upper_bound(&workspace,u);

  // Exitflag
  c_int exitflag = 1; // Start the flag with error

  // Solve Problem
  exitflag = osqp_solve(&workspace); // Flag set to 0 if success

  // Retrieve solution if flag is 0
  if(!exitflag){
    actuatorThrust = workspace.solution->x[0];
    rateDesired.roll = workspace.solution->x[1];
    rateDesired.pitch = workspace.solution->x[2];
    rateDesired.yaw = workspace.solution->x[3];
  }

  // Return exitflag
  return exitflag;
}
#endif // #ifdef OSQP_ENABLED



#ifdef AI_CBF
void __attribute__((used)) DMA1_Stream1_IRQHandler(void){
  DMA_ClearFlag(DMA1_Stream1, UART3_RX_DMA_ALL_FLAGS);
  dma_flag = 1;
}

void USART_DMA_Start(uint32_t baudrate, u_t *aideckRxBuffer, uint32_t BUFFERSIZE){
  // Setup communication
  USART_Config(baudrate, aideckRxBuffer, BUFFERSIZE);
  DMA_ITConfig(USARTx_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
  // Enable DMA USART RX Stream
  DMA_Cmd(USARTx_RX_DMA_STREAM, ENABLE);
  // Enable USART DMA RX Requests
  USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
  // Clear DMA Transfer Complete Flags
  DMA_ClearFlag(USARTx_RX_DMA_STREAM, USARTx_RX_DMA_FLAG_TCIF);
  // Clear USART Transfer Complete Flags
  USART_ClearFlag(USARTx, USART_FLAG_TC);

  DMA_ClearFlag(USARTx_RX_DMA_STREAM, UART3_RX_DMA_ALL_FLAGS);
  NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}
#endif // #ifdef AI_CBF

LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
LOG_ADD(LOG_FLOAT, u_T, &u_T)
LOG_ADD(LOG_FLOAT, u_p, &u_p)
LOG_ADD(LOG_FLOAT, u_q, &u_q)
LOG_ADD(LOG_FLOAT, u_r, &u_r)
LOG_ADD(LOG_FLOAT, pid_T, &pid_T)
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)

PARAM_GROUP_START(controller)
PARAM_ADD(PARAM_UINT8, tiltComp, &tiltCompensationEnabled)
PARAM_GROUP_STOP(controller)
