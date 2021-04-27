
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "sensfusion6.h"
#include "controller_lqr.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

#include "osqp.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)
#define DEG2RAD               (float)M_PI/180.0f
#define RAD2DEG               180.0f/(float)M_PI

static bool tiltCompensationEnabled = false;

static attitude_t rateDesired;
static state_t x_c;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float err_x;
static float err_y;
static float err_z;
static bool flying = false;

void controllerLqrInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
}

bool controllerLqrTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
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

    // Update setpoint x_c
    x_c.position.x = setpoint->position.x;
    x_c.position.y = setpoint->position.y;
    x_c.position.z = setpoint->position.z;
    x_c.attitude.roll = setpoint->attitude.roll;
    x_c.attitude.pitch = setpoint->attitude.pitch;
    x_c.attitude.yaw = capAngle(setpoint->attitude.yaw);
    x_c.velocity.x = setpoint->velocity.x;
    x_c.velocity.y = setpoint->velocity.y;
    x_c.velocity.z = setpoint->velocity.z;

    // Compute error in position
    err_x = state->position.x - x_c.position.x;
    err_y = state->position.y - x_c.position.y;
    err_z = state->position.z - x_c.position.z;

    // DU = -K*(x - x_des)
    actuatorThrust = -(31.6228f*(err_z) + 12.7768f*(state->velocity.z - x_c.velocity.z)); // c (norm thrust)

    rateDesired.roll = -(-4.4721f*(err_y) + 7.6869f*(state->attitude.roll*DEG2RAD - x_c.attitude.roll) - 3.0014f*(state->velocity.y - x_c.velocity.y)); // p

    rateDesired.pitch = -(4.4721f*(err_x) + 7.6869f*(-state->attitude.pitch*DEG2RAD - x_c.attitude.pitch) + 3.0014f*(state->velocity.x - x_c.velocity.x)); // q

    rateDesired.yaw = -(state->attitude.yaw*DEG2RAD - x_c.attitude.yaw); // r

    // Add u_0 | u = DU + u_0
    actuatorThrust += setpoint->thrust;
    rateDesired.roll += setpoint->attitudeRate.roll;
    rateDesired.pitch += setpoint->attitudeRate.pitch;
    rateDesired.yaw += setpoint->attitudeRate.yaw;

    // Apply CBF if enabled
    #ifdef APPLY_CBF
	if(flying){
        apply_cbf(state,100.0f, (float)EPSILON_CBF);
    }
    #endif

    // Saturate thrust and pqr
    actuatorThrust = fmaxf(fminf(actuatorThrust,18), 2);
    rateDesired.roll = fmaxf(fminf(rateDesired.roll,6),-6);
    rateDesired.pitch = fmaxf(fminf(rateDesired.pitch,6),-6);
    rateDesired.yaw = fmaxf(fminf(rateDesired.yaw,6),-6);

    // convert [c, p, q, r] to 0xFFFF and deg/s
    actuatorThrust = 19549.0f*sqrtf(actuatorThrust) - 15159.0f; // Convert to PWM units
    rateDesired.roll = rateDesired.roll*RAD2DEG;
    rateDesired.pitch = rateDesired.pitch*RAD2DEG;
    rateDesired.yaw = rateDesired.yaw*RAD2DEG;

    // Set thrust to 0 if z_desired is 0 and we are close to target
    if((err_x + err_y + err_z) < 0.1f && x_c.position.z == 0)
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

  }
}

int apply_cbf(const state_t *state, const float k, const float epsilon){
    float phi = state->attitude.roll;
    float theta = -state->attitude.pitch; // Opposite sign pitch from estimator

    // Cost function matrix P in CSC format and vector q  (min 1/2*x'Px + f'x)
    c_float P_x[4] = {2.0f, 2.0f, 2.0f, 2.0f, };
    c_int P_nnz = 4;
    c_int P_i[4] = {0, 1, 2, 3, };
    c_int P_p[5] = {0, 1, 2, 3, 4, };
    c_float q[4] = {-2.0f*actuatorThrust, -2.0f*rateDesired.roll,
                    -2.0f*rateDesired.pitch, -2.0f*rateDesired.yaw, }; // -2*u_LQR

    // Constraint matrix A in CSC forma
    float s_phi = arm_sin_f32((float)M_PI*phi/(2.0f*epsilon));
    float s_theta = arm_sin_f32((float)M_PI*theta/(2.0f*epsilon));
    c_float A_x[5] = {s_phi,
                      s_phi*arm_sin_f32(phi)*arm_tan_f32(theta),
                      s_theta*arm_cos_f32(phi),
                      s_phi*arm_cos_f32(phi)*arm_tan_f32(theta),
                      -s_theta*arm_sin_f32(phi), };
    c_int A_nnz = 5;
    c_int A_i[5] = {0, 0, 1, 0, 1, };
    c_int A_p[5] = {0, 0, 1, 3, 5, };

    // Upper and Lower bounds for the constraints
    c_float l[2] = {-999999.9f, -999999.9f, }; // No lower bound
    c_float u[2] = {k*arm_cos_f32((float)M_PI*state->attitude.roll/(2.0f*epsilon)),
                    k*arm_cos_f32((float)M_PI*-state->attitude.pitch/(2.0f*epsilon)), }; // Upper bound NOTE pitch is opposite

    // Problem size
    c_int n = 4;
    c_int m = 2;

	// Exitflag
    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

 	// Populate data
    if (data) {
        data->n = n;
        data->m = m;
        data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
        data->q = q;
        data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
        data->l = l;
        data->u = u;
    }

	// Define solver settings as default
    if (settings) {
        osqp_set_default_settings(settings);
        settings->alpha = 1.0; // Change alpha parameter
    }

	// Setup workspace
    exitflag = osqp_setup(&work, data, settings);

    // Solve Problem
    osqp_solve(work);

    // Retrieve solution if flag is 0
    if(!exitflag){
        actuatorThrust = work->solution.x[0];
        rateDesired.roll = work->solution.x[1];
        rateDesired.pitch = work->solution.x[2];
        rateDesired.yaw = work->solution.x[3];
    }

    // Cleanup
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings) c_free(settings);

    // Return exitflag
    return exitflag;
}


LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)

PARAM_GROUP_START(controller)
PARAM_ADD(PARAM_UINT8, tiltComp, &tiltCompensationEnabled)
PARAM_GROUP_STOP(controller)
