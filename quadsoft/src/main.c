/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/pwm.h>
#include <math.h>

#define PWM_CHANNEL 1
#define PWM_CTLR_NODE DT_NODELABEL(pwm2)

#define X 0
#define Y 1
#define Z 2
#define NORTH 0
#define EAST 1
#define DOWN 2
#define ROLL 0
#define PITCH 1
#define YAW 2
#define PI 3.14159265358979323846
#define RAD2DEG 180.0 / math.PI
#define DEG2RAD math.pi / 180.0
#define RAD_TO_DEG (180.0 / PI)
#define DEG_TO_RAD (PI / 180.0)
#define ACCELEROMETER_CUTOFF 100
#define COMPLEMENTARY_FILTER_ALPHA 0.98
// State vector: [position, velocity, orientation]
#define STATE_DIM 9
// Measurement vector: [accelerometer, gyroscope, magnetometer]
#define MEASURE_DIM 9


typedef struct {
	double P[4][4];	// Prediction error covariance matrix
	double Q[4][4];  // Process noise covariance matrix
	double R[6][6];	// Measurement noise covariance matrix

	double K[4][6];

	double x[4];
	double xp[4];

	double Pp[4][4];

	// Magnetic Vector References
	double ref_mx;
	double ref_my;
	double ref_mz;
} ekf_t;


void EKF_init(ekf_t* ekf, double ref_mx, double ref_my, double ref_mz, double N_Q, double N_P, double N_R);

void EKF_update(ekf_t* ekf, double euler[3], double ax, double ay, double az, double p, double q, double r, double mx, double my, double mz, double dt);

uint8_t inverse_matrix(double a[6][6], double a_inv[6][6]);

void EKFquaternionToEuler(double q[4], double euler[3]);


void EKF_init(ekf_t* ekf, double ref_mx, double ref_my, double ref_mz, double N_Q, double N_P, double N_R) {
	// Initialization:
	// Prediction error covariance matrix
	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t j = 0; j < 4; j++) {
			if (i == j) {
				ekf->P[i][j] = N_P;
			}
			else {
				ekf->P[i][j] = 0.0f;
			}
		}
	}

	// Process noise covariance matrix
	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t j = 0; j < 4; j++) {
			if (i == j) {
				ekf->Q[i][j] = N_Q;
			}
			else {
				ekf->Q[i][j] = 0.0f;
			}
		}
	}

	// Measurement noise covariance matrix
	for (uint8_t i = 0; i < 6; i++) {
		for (uint8_t j = 0; j < 6; j++) {
			if (i == j) {
				ekf->R[i][j] = N_R;
			}
			else {
				ekf->R[i][j] = 0.0f;
			}
		}
	}

	ekf->x[0] = 1;
	ekf->x[1] = 0;
	ekf->x[2] = 0;
	ekf->x[3] = 0;


	// Normalize Reference Magnetic Vector
	double M = sqrtf(ref_mx * ref_mx + ref_my * ref_my + ref_mz * ref_mz);
	ekf->ref_mx = ref_mx / M;
	ekf->ref_my = ref_my / M;
	ekf->ref_mz = ref_mz / M;
}


void EKF_update(ekf_t* ekf, double euler[3], double ax, double ay, double az, double p, double q, double r, double mx, double my, double mz, double dt) {
	// Variable Definitions
	double F[4][4];	// Jacobian matrix of F
	double H[6][4];	// Jacobian matrix of H
	double FP[4][4];
	double FPFt[4][4];
	double HPp[6][4];
	double HPpHt[6][6];
	double PpHt[4][6];
	double S_inv[6][6];
	double Hxp[6];
	double z[6];
	double zmHxp[6];
	double KzmHxp[4];
	double KH[4][4];
	double KHPp[4][4];
	double G;
	double M;
	uint8_t mat_error;


	// Normalization
	G = sqrtf(ax * ax + ay * ay + az * az);
	M = sqrtf(powf(mx, 2) + powf(my, 2) + powf(mz, 2));
	ax = ax / G;
	ay = ay / G;
	az = az / G;
	mx = mx / M;
	my = my / M;
	mz = mz / M;


	// Calculate the Jacobian matrix of F
	F[0][0] = 1;
	F[0][1] = -p * dt / 2;
	F[0][2] = -q * dt / 2;
	F[0][3] = -r * dt / 2;

	F[1][0] = p * dt / 2;
	F[1][1] = 1;
	F[1][2] = r * dt / 2;
	F[1][3] = -q * dt / 2;

	F[2][0] = q * dt / 2;
	F[2][1] = -r * dt / 2;
	F[2][2] = 1;
	F[2][3] = p * dt / 2;

	F[3][0] = r * dt / 2;
	F[3][1] = q * dt / 2;
	F[3][2] = -p * dt / 2;
	F[3][3] = 1;


	// Prediction of x
	ekf->xp[0] = F[0][0] * ekf->x[0] + F[0][1] * ekf->x[1] + F[0][2] * ekf->x[2] + F[0][3] * ekf->x[3];
	ekf->xp[1] = F[1][0] * ekf->x[0] + F[1][1] * ekf->x[1] + F[1][2] * ekf->x[2] + F[1][3] * ekf->x[3];
	ekf->xp[2] = F[2][0] * ekf->x[0] + F[2][1] * ekf->x[1] + F[2][2] * ekf->x[2] + F[2][3] * ekf->x[3];
	ekf->xp[3] = F[3][0] * ekf->x[0] + F[3][1] * ekf->x[1] + F[3][2] * ekf->x[2] + F[3][3] * ekf->x[3];


	// Prediction of P
	/* Pp = F*P*F' + Q; */
	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t j = 0; j < 4; j++) {
			FP[i][j] = F[i][0] * ekf->P[0][j] + F[i][1] * ekf->P[1][j] + F[i][2] * ekf->P[2][j] + F[i][3] * ekf->P[3][j];
		}
	}

	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t j = 0; j < 4; j++) {
			FPFt[i][j] = FP[i][0] * F[j][0] + FP[i][1] * F[j][1] + FP[i][2] * F[j][2] + FP[i][3] * F[j][3];
		}
	}

	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t j = 0; j < 4; j++) {
			ekf->Pp[i][j] = FPFt[i][j] + ekf->Q[i][j];
		}
	}


	// Calculate the Jacobian matrix of h
	H[0][0] = -ekf->x[2];
	H[0][1] = ekf->x[3];
	H[0][2] = -ekf->x[0];
	H[0][3] = ekf->x[1];

	H[1][0] = ekf->x[1];
	H[1][1] = ekf->x[0];
	H[1][2] = ekf->x[3];
	H[1][3] = ekf->x[2];

	H[2][0] = ekf->x[0];
	H[2][1] = -ekf->x[1];
	H[2][2] = -ekf->x[2];
	H[2][3] = ekf->x[3];

	H[3][0] = ekf->x[0] * ekf->ref_mx + ekf->x[3] * ekf->ref_my - ekf->x[2] * ekf->ref_mz;
	H[3][1] = ekf->x[1] * ekf->ref_mx + ekf->x[2] * ekf->ref_my + ekf->x[3] * ekf->ref_mz;
	H[3][2] = -ekf->x[2] * ekf->ref_mx + ekf->x[1] * ekf->ref_my - ekf->x[0] * ekf->ref_mz;
	H[3][3] = -ekf->x[3] * ekf->ref_mx + ekf->x[0] * ekf->ref_my + ekf->x[1] * ekf->ref_mz;

	H[4][0] = -ekf->x[3] * ekf->ref_mx + ekf->x[0] * ekf->ref_my + ekf->x[1] * ekf->ref_mz;
	H[4][1] = ekf->x[2] * ekf->ref_mx - ekf->x[1] * ekf->ref_my + ekf->x[0] * ekf->ref_mz;
	H[4][2] = ekf->x[1] * ekf->ref_mx + ekf->x[2] * ekf->ref_my + ekf->x[3] * ekf->ref_mz;
	H[4][3] = -ekf->x[0] * ekf->ref_mx - ekf->x[3] * ekf->ref_my + ekf->x[2] * ekf->ref_mz;

	H[5][0] = ekf->x[2] * ekf->ref_mx - ekf->x[1] * ekf->ref_my + ekf->x[0] * ekf->ref_mz;
	H[5][1] = ekf->x[3] * ekf->ref_mx - ekf->x[0] * ekf->ref_my - ekf->x[1] * ekf->ref_mz;
	H[5][2] = ekf->x[0] * ekf->ref_mx + ekf->x[3] * ekf->ref_my - ekf->x[2] * ekf->ref_mz;
	H[5][3] = ekf->x[1] * ekf->ref_mx + ekf->x[2] * ekf->ref_my + ekf->x[3] * ekf->ref_mz;


	// S = (H*Pp*H' + R);
	// H*Pp
	for (uint8_t i = 0; i < 6; i++) {
		for (uint8_t j = 0; j < 4; j++) {
			HPp[i][j] = H[i][0] * ekf->Pp[0][j] + H[i][1] * ekf->Pp[1][j] + H[i][2] * ekf->Pp[2][j] + H[i][3] * ekf->Pp[3][j];
		}
	}

	// H*Pp*H'
	for (uint8_t i = 0; i < 6; i++) {
		for (uint8_t j = 0; j < 6; j++) {
			HPpHt[i][j] = HPp[i][0] * H[j][0] + HPp[i][1] * H[j][1] + HPp[i][2] * H[j][2] + HPp[i][3] * H[j][3];
		}
	}

	// H*Pp*H' + R
	for (uint8_t i = 0; i < 6; i++) {
		for (uint8_t j = 0; j < 6; j++) {
			HPpHt[i][j] = HPpHt[i][j] + ekf->R[i][j]; // S
		}
	}


	// K = Pp*H'*(S.inverse);
	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t j = 0; j < 6; j++) {
			PpHt[i][j] = ekf->Pp[i][0] * H[j][0] + ekf->Pp[i][1] * H[j][1] + ekf->Pp[i][2] * H[j][2] + ekf->Pp[i][3] * H[j][3];
		}
	}

	mat_error = inverse_matrix(HPpHt, S_inv);
	if (!mat_error) {
		return;
	}

	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t j = 0; j < 6; j++) {
			ekf->K[i][j] = PpHt[i][0] * S_inv[0][j] + PpHt[i][1] * S_inv[1][j] + PpHt[i][2] * S_inv[2][j] + PpHt[i][3] * S_inv[3][j] + PpHt[i][4] * S_inv[4][j] + PpHt[i][5] * S_inv[5][j];
		}
	}


	// x = xp + K*(z - H*xp);
	// H*xp
	for (uint8_t i = 0; i < 6; i++) {
		Hxp[i] = ekf->xp[0] * H[i][0] + ekf->xp[1] * H[i][1] + ekf->xp[2] * H[i][2] + ekf->xp[3] * H[i][3];
	}

	z[0] = ax;
	z[1] = ay;
	z[2] = az;
	z[3] = mx;
	z[4] = my;
	z[5] = mz;

	// (z - H*xp)
	for (uint8_t i = 0; i < 6; i++) {
		zmHxp[i] = (z[i] - Hxp[i]);
	}

	// K*(z - H*xp)
	for (uint8_t i = 0; i < 4; i++) {
		KzmHxp[i] = 0;
		for (uint8_t j = 0; j < 6; j++) {
			KzmHxp[i] += ekf->K[i][j] * zmHxp[j];
		}
	}

	// xp + K*(z - H*xp)
	for (uint8_t i = 0; i < 4; i++) {
		ekf->x[i] = ekf->xp[i] + KzmHxp[i];
	}


	// P = Pp - K*H*Pp;
	// K*H
	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t j = 0; j < 4; j++) {
			KH[i][j] = ekf->K[i][0] * H[0][j] + ekf->K[i][1] * H[1][j] + ekf->K[i][2] * H[2][j] + ekf->K[i][3] * H[3][j] + ekf->K[i][4] * H[4][j] + ekf->K[i][5] * H[5][j];
		}
	}

	// K*H*Pp
	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t j = 0; j < 4; j++) {
			KHPp[i][j] = KH[i][0] * ekf->Pp[0][j] + KH[i][1] * ekf->Pp[1][j] + KH[i][2] * ekf->Pp[2][j] + KH[i][3] * ekf->Pp[3][j];
		}
	}

	// P = Pp - K*H*Pp
	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t j = 0; j < 4; j++) {
			ekf->P[i][j] = ekf->Pp[i][j] - KHPp[i][j];
		}
	}


	EKFquaternionToEuler(ekf->x, euler);

}

uint8_t inverse_matrix(double a[6][6], double a_inv[6][6]) {
	// Initialize matrix
	for (uint8_t i = 0; i < 6; i++) {
		for (uint8_t j = 0; j < 6; j++) {
			if (i != j)
				a_inv[i][j] = 0;
			else
				a_inv[i][j] = 1;
		}
	}

	// Applying Gauss Jordan Elimination
	double ratio = 1.0;
	for (uint8_t i = 0; i < 6; i++)
	{
		if (a[i][i] == 0.0)
		{
			return 0;
		}
		for (uint8_t j = 0; j < 6; j++)
		{
			if (i != j)
			{
				ratio = a[j][i] / a[i][i];
				for (uint8_t k = 0; k < 6; k++)
				{
					a[j][k] = a[j][k] - ratio * a[i][k];
					a_inv[j][k] = a_inv[j][k] - ratio * a_inv[i][k];
				}
			}
		}
	}
	// Row Operation to Make Principal Diagonal to 1
	for (uint8_t i = 0; i < 6; i++)
	{
		for (uint8_t j = 0; j < 6; j++)
		{
			a_inv[i][j] = a_inv[i][j] / a[i][i];
		}
	}
	return 1;
}

void EKFquaternionToEuler(double q[4], double euler[3]) {
	double phi = atan2f(2 * (q[2] * q[3] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));

	double sinp = 2 * (q[1] * q[3] - q[0] * q[2]);
	double theta = 0.0;

	if (fabsf(sinp) >= 1) {
		theta = (sinp >= 0) ? PI / 2 : -PI / 2;
	}
	else {
		theta = -asinf(sinp);
	}

	double psi = atan2f(2 * (q[1] * q[2] + q[0] * q[3]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));

	euler[0] = phi;
	euler[1] = theta;
	euler[2] = psi;
}

// Global variables
double acc_a_x = 0;
double acc_a_y = 0;
double acc_a_z = 0;
double acc_v_x = 0;
double acc_v_y = 0;
double acc_v_z = 0;
double acc_s_x = 0;
double acc_s_y = 0;
double acc_s_z = 0;

double gyro_a_x = 0;
double gyro_a_y = 0;
double gyro_a_z = 0;
double gyro_v_x = 0;
double gyro_v_y = 0;
double gyro_v_z = 0;
double gyro_s_x = 0;
double gyro_s_y = 0;
double gyro_s_z = 0;
double mag_a[3];
double delta_t = 0.001;
double t = 0;
double pos_p_gain[3] = {1.0, 1.0, 1.0};
double vel_p_gain[3] = {5.0, 5.0, 4.0};
double vel_d_gain[3] = {0.5, 0.5, 0.5};
double vel_i_gain[3] = {5.0, 5.0, 5.0};
double attitute_p_gain[3] = {8.0, 8.0, 1.5};
double Pp = 1.5;
double Dp = 0.4;
double Pq = 1.5;
double Dq = 0.4;
double Pr = 1.0;
double Dr = 0.1;
double rate_p_gain[3] = {1.5, 0.4, 1.5};
double rate_d_gain[3] = {0.4, 1.0, 0.1};
double uMax = 5.0;
double vMax = 5.0;
double wMax = 5.0;
double pMax = 3.5;
double qMax = 3.5;
double rMax = 2.62;
double velMax[3] = {5.0, 5.0, 5.0};
double vMaxAll = 5.0;
double rateMax[3] = {3.5, 3.5, 2.62};
double thr_int[3];
double tiltMax = 0.8;
int saturateVel_separetely = 0;
double gravitation_constant = 9.81;
double quadcopter_mass = 1.0;
double rate_control[3] = {0.0, 0.0, 0.0};
double minWMotor = 0.0;
double maxWMotor = 1000.0;
double minThr = 0.0;
double maxThr = 1000.0;
double q_target[4] = {1.0, 0.0, 0.0, 0.0};
double acc_setpoint[3];
double yawW = 0.0;
double rate_error[3];
double roll_pitch_gain = 0.0; 
double pos_sp[3] = {0.0, 0.0, 0.0};
double vel_sp[3] = {1.0, 0.0, 0.0};
double acc_sp[3] = {0.0, 0.0, 0.0};
double thrust_sp[3] = {0.0, 0.0, 0.0};
double eul_sp[3] = {0.0, 0.0, 0.0};
double pqr_sp[3] = {0.0, 0.0, 0.0};
double yawFF[3] = {0.0, 0.0, 0.0};
double gain = 0;
double yaw_w;
double v_error[3];
double v_error_lim[3];
double body_x[3];
double body_y[3];
double body_z[3];
double rate_setpoint[4];
double yaw_setpoint;
double y_C[3];
double quad_dcm[3][3];
double quad_quat_state[4] = {1.0, 0.0, 0.0, 0.0};
double e_z[3];
double e_z_d[3];
double qe_red[4];
double qd_red[4];
double qd[4];
double result[4];
double qd_red_inverse[4]; 
double q_mix[4];
double quadternion_error[4];
double quad_quat_state_inverse[4];
double thrust_sp_norm;
double w_cmd_clipped[4];
double w_cmd[4];

// Normierte Werte
double qd_norm;
double e_z_norm;
double nor;
double norm;
double qe_norm;

// Attitude structure
typedef struct {
    double roll;
    double pitch;
    double yaw;
} Attitude;

Attitude attitude = {0};
double gyro_bias[3] = {0};

// Function to get the current attitude
Attitude get_attitude() {
    return attitude;
}

// Function to calculate arctangent in all quadrants
double atan2_approx(double y, double x) {
    if (x == 0.0) {
        if (y > 0.0) return PI / 2;
        if (y < 0.0) return -PI / 2;
        return 0.0;
    }
    double atan;
    double z = y / x;
    if (fabs(z) < 1.0) {
        atan = z / (1.0 + 0.28 * z * z);
        if (x < 0.0) {
            if (y < 0.0) atan -= PI;
            else atan += PI;
        }
    } else {
        atan = PI / 2.0 - z / (z * z + 0.28);
        if (y < 0.0) atan -= PI;
    }
    return atan;
}

// Update the filter
void update_complementary_filter(
	double acc_x, 
	double acc_y, 
	double acc_z,
    double gyro_x, 
	double gyro_y, 
	double gyro_z,
    double mag_x, 
	double mag_y, 
	double mag_z,
    double dt) {
		
    // Calculate accelerometer angles
    double accel_roll = atan2_approx(
		acc_y, 
		acc_z
	);
    double accel_pitch = atan2_approx(
		-acc_x, 
		sqrtf(acc_y * acc_y + acc_z * acc_z)
	);

    // Integrate gyroscope readings
    double gyro_roll = attitude.roll + (
		gyro_x - gyro_bias[0]
	) * dt;
    double gyro_pitch = attitude.pitch + (
		gyro_y - gyro_bias[1]
	) * dt;
    double gyro_yaw = attitude.yaw + (
		gyro_z - gyro_bias[2]
	) * dt;

    // Combine accelerometer and gyroscope for roll and pitch
    attitude.roll = COMPLEMENTARY_FILTER_ALPHA * gyro_roll + (1 - COMPLEMENTARY_FILTER_ALPHA) * accel_roll;
    attitude.pitch = COMPLEMENTARY_FILTER_ALPHA * gyro_pitch + (1 - COMPLEMENTARY_FILTER_ALPHA) * accel_pitch;

    // Calculate magnetometer yaw
    double mag_x_comp; 
	mag_x_comp = mag_x * cos(attitude.pitch) + mag_z * sin(attitude.pitch);
    
	double mag_y_comp;
	mag_y_comp = mag_x * sin(attitude.roll) * sin(attitude.pitch) + mag_y * cos(attitude.roll) - mag_z * sin(attitude.roll) * cos(attitude.pitch);
    double mag_yaw = atan2_approx(-mag_y_comp, mag_x_comp);

    // Combine gyroscope and magnetometer for yaw
    attitude.yaw = COMPLEMENTARY_FILTER_ALPHA * gyro_yaw + (1 - COMPLEMENTARY_FILTER_ALPHA) * mag_yaw;

    // Normalize yaw to [-PI, PI]
	while (attitude.yaw > PI) attitude.yaw -= 2 * PI;
    while (attitude.yaw < -PI) attitude.yaw += 2 * PI;

    // Update gyro bias (simple low-pass filter)
    gyro_bias[0] += 0.0001 * (gyro_x - gyro_bias[0]);
    gyro_bias[1] += 0.0001 * (gyro_y - gyro_bias[1]);
    gyro_bias[2] += 0.0001 * (gyro_z - gyro_bias[2]);
}

// Initialize the filter
void init_complementary_filter() {
    attitude.roll = 0;
    attitude.pitch = 0;
    attitude.yaw = 0;
}


#ifdef CONFIG_LPS2XDF_TRIGGER
static int lps22df_trig_cnt;

static void lps22df_trigger_handler(
	const struct device *dev,
	const struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
	lps22df_trig_cnt++;
}
#endif

#ifdef CONFIG_LIS2MDL_TRIGGER
static int lis2mdl_trig_cnt;

static void lis2mdl_trigger_handler(
	const struct device *dev,
	const struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
	lis2mdl_trig_cnt++;
}
#endif

#ifdef CONFIG_LSM6DSO16IS_TRIGGER
static int lsm6dso16is_acc_trig_cnt;

static void lsm6dso16is_acc_trig_handler(
	const struct device *dev,
	const struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
	lsm6dso16is_acc_trig_cnt++;
}
#endif

#ifdef CONFIG_LSM6DSV16X_TRIGGER
static int lsm6dsv16x_acc_trig_cnt;

static void lsm6dsv16x_acc_trig_handler(
	const struct device *dev,
	const struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
	lsm6dsv16x_acc_trig_cnt++;
}
#endif

static void lis2mdl_config(const struct device *lis2mdl)
{
	struct sensor_value odr_attr;

	/* set LIS2MDL sampling frequency to 100 Hz */
	odr_attr.val1 = 100;
	odr_attr.val2 = 0;

	if (sensor_attr_set(
		lis2mdl,
		SENSOR_CHAN_ALL,
		SENSOR_ATTR_SAMPLING_FREQUENCY, 
		&odr_attr) < 0) {

		printk(
			"Cannot set sampling frequency for LIS2MDL\n"
		);
		return;
	}

#ifdef CONFIG_LIS2MDL_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_MAGN_XYZ;
	sensor_trigger_set(
		lis2mdl, 
		&trig, 
		lis2mdl_trigger_handler
	);
#endif
}

static void lsm6dso16is_config(
	const struct device *lsm6dso16is)
{
	struct sensor_value odr_attr, fs_attr, mode_attr;

	mode_attr.val1 = 0; /* HP */

	if (sensor_attr_set(lsm6dso16is, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_CONFIGURATION, &mode_attr) < 0) {
		printk("Cannot set mode for LSM6DSO16IS accel\n");
		return;
	}

	/* set LSM6DSO16IS accel sampling frequency to 208 Hz */
	odr_attr.val1 = 208;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dso16is, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for LSM6DSO16IS accel\n");
		return;
	}

	sensor_g_to_ms2(16, &fs_attr);

	if (sensor_attr_set(
		lsm6dso16is, 
		SENSOR_CHAN_ACCEL_XYZ,
		SENSOR_ATTR_FULL_SCALE, 
		&fs_attr) < 0) {
		
		printk(
			"Cannot set full scale for LSM6DSO16IS accel\n"
		);
		return;
	}

	/* set LSM6DSO16IS gyro sampling frequency to 208 Hz */
	odr_attr.val1 = 208;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dso16is, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk(
			"Cannot set sampling frequency for LSM6DSO16IS gyro\n"
		);
		return;
	}

	sensor_degrees_to_rad(250, &fs_attr);

	if (sensor_attr_set(lsm6dso16is, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
		printk("Cannot set full scale for LSM6DSO16IS gyro\n");
		return;
	}

#ifdef CONFIG_LSM6DSO16IS_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(lsm6dso16is, &trig, lsm6dso16is_acc_trig_handler);
#endif
}

static void lsm6dsv16x_config(
	const struct device *lsm6dsv16x)
{
	struct sensor_value odr_attr, fs_attr, mode_attr;

	mode_attr.val1 = 0; /* HP */

	if (sensor_attr_set(
		lsm6dsv16x, 
		SENSOR_CHAN_ACCEL_XYZ,
		SENSOR_ATTR_CONFIGURATION, 
		&mode_attr) < 0) {
			
		printk("Cannot set mode for LSM6DSV16X accel\n");
		return;
	}

	/* set LSM6DSV16X accel sampling frequency to 208 Hz */
	odr_attr.val1 = 208;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dsv16x, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk(
			"Cannot set sampling frequency for LSM6DSV16X accel\n"
		);
		return;
	}

	sensor_g_to_ms2(16, &fs_attr);

	if (sensor_attr_set(lsm6dsv16x, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
		printk("Cannot set full scale for LSM6DSV16X accel\n");
		return;
	}

	/* set LSM6DSV16X gyro sampling frequency to 208 Hz */
	odr_attr.val1 = 208;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dsv16x, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for LSM6DSV16X gyro\n");
		return;
	}

	sensor_degrees_to_rad(250, &fs_attr);

	if (sensor_attr_set(lsm6dsv16x, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
		printk(
			"Cannot set full scale for LSM6DSV16X gyro\n"
		);
		return;
	}

#ifdef CONFIG_LSM6DSV16X_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(lsm6dsv16x, &trig, lsm6dsv16x_acc_trig_handler);
#endif
}

static void lps22df_config(const struct device *lps22df)
{
	struct sensor_value odr_attr;

	/* set LPS22DF accel sampling frequency to 10 Hz */
	odr_attr.val1 = 10;
	odr_attr.val2 = 0;

	if (sensor_attr_set(
		lps22df, 
		SENSOR_CHAN_ALL,
		SENSOR_ATTR_SAMPLING_FREQUENCY, 
		&odr_attr) < 0) {

		printk(
			"Cannot set sampling frequency for LPS22DF accel\n"
		);
		return;
	}

#ifdef CONFIG_LPS2XDF_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ALL;
	sensor_trigger_set(lps22df, &trig, lps22df_trigger_handler);
#endif
}

int main(void)
{

	const struct device *pwm_dev = DEVICE_DT_GET(
		PWM_CTLR_NODE
		);
    
    if (!device_is_ready(pwm_dev)) {
        printk("Error: PWM device not ready\n");
        return -1;
    }

	struct sensor_value lis2mdl_magn[3], lis2mdl_temp, lps22df_press, lps22df_temp;
	struct sensor_value lsm6dso16is_xl[3], lsm6dso16is_gy[3];

#ifdef CONFIG_LSM6DSO16IS_ENABLE_TEMP
	struct sensor_value lsm6dso16is_temp;
#endif
#ifdef CONFIG_LSM6DSV16X_ENABLE_TEMP
	struct sensor_value lsm6dsv16x_temp;
#endif

	struct sensor_value lsm6dsv16x_xl[3];
	struct sensor_value lsm6dsv16x_gy[3];
	int cnt = 1;

	const struct device *const lis2mdl = DEVICE_DT_GET_ONE(
		st_lis2mdl
	);
	const struct device *const lsm6dso16is = DEVICE_DT_GET_ONE(st_lsm6dso16is);
	const struct device *const lsm6dsv16x = DEVICE_DT_GET_ONE(st_lsm6dsv16x);
	const struct device *const lps22df = DEVICE_DT_GET_ONE(st_lps22df);
	
	if (!device_is_ready(lsm6dso16is)) {
		printk(
			"%s: device not ready.\n",
			lsm6dso16is->name
		);
		return 0;
	}
	if (!device_is_ready(lsm6dsv16x)) {
		printk(
			"%s: device not ready.\n",
			lsm6dsv16x->name
		);
		return 0;
	}
	if (!device_is_ready(lis2mdl)) {
		printk(
			"%s: device not ready.\n", 
			lis2mdl->name
		);
		return 0;
	}
	if (!device_is_ready(lps22df)) {
		printk(
			"%s: device not ready.\n", 
			lps22df->name
		);
		return 0;
	}

	lis2mdl_config(lis2mdl);
	lsm6dso16is_config(lsm6dso16is);
	lsm6dsv16x_config(lsm6dsv16x);
	lps22df_config(lps22df);

    int64_t last_sample_time_ms = k_uptime_get();

	struct euler_angles {
		double roll;
		double pitch;
		double yaw;
	};

	static struct euler_angles attitude = {0};

	init_complementary_filter();

	roll_pitch_gain = (0.5 * (
        attitute_p_gain[X] + 
        attitute_p_gain[Y]
    ));
    yaw_w = attitute_p_gain[Z] / roll_pitch_gain;

    if (yaw_w < 0.0) {
        yaw_w = 0.0;
    }
    if (yaw_w > 1.0) {
        yaw_w = 1.0;
    }

    thr_int[0] = 0.0;
    thr_int[1] = 0.0;
    thr_int[2] = 0.0;

    attitute_p_gain[2] = roll_pitch_gain;

    double v_target[3] = {0.0, 0.0, 0.0};

    double quad_a[3];
    double quad_v[3];

    double rate_setpoint[3];
    double quad_rate[3] = {0.0, 0.0, 0.0};
    double omega_dot[3] = {0.0, 0.0, 0.0};
    double rateCtrl[3];

    double mixerFM[4][4] = {
        {  23000,  64000,  64000, -1530000},
        {  23000, -64000,  64000,  1530000},
        {  23000, -64000, -64000, -1530000},
        {  23000,  64000, -64000,  1530000}
    };

	ekf_t ekf;
	double euler_ekf[3];

	mag_a[X] = 1.0;
	mag_a[Y] = 1.0;
	mag_a[Z] = -9.81;

	// EKF_init(
	// 	&ekf, 
	// 	mag_a[X], 
	// 	mag_a[Y], 
	// 	mag_a[Z], 
	// 	0.1, 
	// 	1, 
	// 	100
	// );

	while (1) {

#ifndef CONFIG_LIS2MDL_TRIGGER
		if (sensor_sample_fetch(lis2mdl) < 0) {
			printf("LIS2MDL Magn Sensor sample update error\n");
			return 0;
		}
#endif
#ifndef CONFIG_LSM6DSO16IS_TRIGGER
		if (sensor_sample_fetch(lsm6dso16is) < 0) {
			printf("LSM6DSO16IS Sensor sample update error\n");
			return 0;
		}
#endif
#ifndef CONFIG_LSM6DSV16X_TRIGGER
		if (sensor_sample_fetch(lsm6dsv16x) < 0) {
			printf("LSM6DSV16X Sensor sample update error\n");
			return 0;
		}
#endif
#ifndef CONFIG_LPS2XDF_TRIGGER
		if (sensor_sample_fetch(lps22df) < 0) {
			printf("LPS22DF pressure sample update error\n");
			return 0;
		}
#endif

		sensor_channel_get(lis2mdl, SENSOR_CHAN_MAGN_XYZ, lis2mdl_magn);
		sensor_channel_get(lis2mdl, SENSOR_CHAN_DIE_TEMP, &lis2mdl_temp);
		sensor_channel_get(lsm6dso16is, SENSOR_CHAN_ACCEL_XYZ, lsm6dso16is_xl);
		sensor_channel_get(lsm6dso16is, SENSOR_CHAN_GYRO_XYZ, lsm6dso16is_gy);
#ifdef CONFIG_LSM6DSO16IS_ENABLE_TEMP
		sensor_channel_get(lsm6dso16is, SENSOR_CHAN_DIE_TEMP, &lsm6dso16is_temp);
#endif
#ifdef CONFIG_LSM6DSV16X_ENABLE_TEMP
		sensor_channel_get(lsm6dsv16x, SENSOR_CHAN_DIE_TEMP, &lsm6dsv16x_temp);
#endif
		sensor_channel_get(
			lsm6dsv16x, 
			SENSOR_CHAN_ACCEL_XYZ, 
			lsm6dsv16x_xl
		);
		sensor_channel_get(
			lsm6dsv16x, 
			SENSOR_CHAN_GYRO_XYZ, 
			lsm6dsv16x_gy
		);
		sensor_channel_get(
			lps22df, 
			SENSOR_CHAN_PRESS, 
			&lps22df_press
		);
		sensor_channel_get(
			lps22df, 
			SENSOR_CHAN_AMBIENT_TEMP, 
			&lps22df_temp
		);

		/* Display sensor data */

		/* Erase previous */
		// /printf("\0033\014");

		/*

		printf("X-NUCLEO-IKS4A1 sensor dashboard\n\n");

		printf(
			"LIS2MDL: Magn (gauss): 
			x: %.3f, 
			y: %.3f, 
			z: %.3f\n",
		    sensor_value_to_double(&lis2mdl_magn[0]),
		    sensor_value_to_double(&lis2mdl_magn[1]),
		    sensor_value_to_double(&lis2mdl_magn[2])
		);

		printf(
			"LIS2MDL: Temperature: %.1f C\n",
		    sensor_value_to_double(&lis2mdl_temp)
		);

		*/

		/*
		printf("LSM6DSO16IS: Temperature: %.1f C\n",
		       sensor_value_to_double(&lsm6dso16is_temp));
		*/
		
		int64_t current_time_ms = k_uptime_get();   
		double dt = (
			current_time_ms - last_sample_time_ms
		) / 1000.0f;
        last_sample_time_ms = current_time_ms;

		// Gyro
		acc_a_x = (
			sensor_value_to_double(&lsm6dsv16x_xl[X]) + 
			sensor_value_to_double(&lsm6dso16is_xl[X])
		) / 2;
		acc_a_y = (
			sensor_value_to_double(&lsm6dsv16x_xl[Y]) +
			sensor_value_to_double(&lsm6dso16is_xl[Y])
		) / 2;
		acc_a_z = (
			sensor_value_to_double(&lsm6dsv16x_xl[Z]) + 
			sensor_value_to_double(&lsm6dso16is_xl[Z])
		) / 2 - 9.81;

		// if (abs(acc_a_x) < ACCELEROMETER_CUTOFF) {
		// 	acc_a_x = 0.0;
		// } 
		// if (abs(acc_a_y) < ACCELEROMETER_CUTOFF) {
		// 	acc_a_y = 0.0;
		// } 
		// if (abs(acc_a_z) < ACCELEROMETER_CUTOFF) {
		// 	acc_a_z = 0.0;
		// } 

		acc_v_x = acc_v_x + dt * (
			acc_a_x
		);
		acc_v_y = acc_v_y + dt * (
			acc_a_y
		);
		acc_v_z = acc_v_z + dt * (
			acc_a_z
		);


		// acc_s_x = acc_s_x + dt * (
		// 	acc_v_x
		// );
		// acc_s_y = acc_s_y + dt * (
		// 	acc_v_y
		// );
		// acc_s_z = acc_s_z + dt * (
		// 	acc_v_z
		// );

		// printf(
		// 	"LSM6DSV16X: Accel (m.s-2): 
		// 	x: %.3f, 
		// 	y: %.3f, 
		// 	z: %.3f\n",
		// 	sensor_value_to_double(&lsm6dsv16x_xl[0]),
		// 	sensor_value_to_double(&lsm6dsv16x_xl[1]),
		// 	sensor_value_to_double(&lsm6dsv16x_xl[2])
		// );

		printf("accx: %.4f\n", acc_v_x);
		printf("accy: %.4f\n", acc_v_y);
		printf("accz: %.4f\n", acc_v_z);

		mag_a[X] = sensor_value_to_double(
			&lis2mdl_magn[X]
		); 
		mag_a[Y] = sensor_value_to_double(
			&lis2mdl_magn[Y]
		); 
		mag_a[Z] = sensor_value_to_double(
			&lis2mdl_magn[Z]
		);

		// Gyro
		gyro_v_x = (
			sensor_value_to_double(&lsm6dsv16x_gy[ROLL]) + sensor_value_to_double(&lsm6dso16is_gy[ROLL])
		) / 2;
		gyro_v_y = (
			sensor_value_to_double(&lsm6dsv16x_gy[PITCH]) + 
			sensor_value_to_double(&lsm6dso16is_gy[PITCH])
		) / 2;
		gyro_v_z = (
			sensor_value_to_double(&lsm6dsv16x_gy[YAW]) + sensor_value_to_double(&lsm6dso16is_gy[YAW])
		) / 2;

		// if (abs(gyro_v_x) < 0.001) {
		// 	gyro_v_x = 0.0;
		// } 
		// if (abs(gyro_v_y) < 0.001) {
		// 	gyro_v_y = 0.0;
		// } 
		// if (abs(gyro_v_z) < 0.001) {
		// 	gyro_v_z = 0.0;
		// } 

		gyro_s_x = gyro_s_x + dt * (
			gyro_v_x
		);
		gyro_s_y = gyro_s_y + dt * (
			gyro_v_y
		);
		gyro_s_z = gyro_s_z + dt * (
			gyro_v_z
		);

		// printf("LSM6DSV16X: GYro (dps): x: %.8f, y: %.8f, 	z: %.8f\n",
		// 	sensor_value_to_double(&lsm6dsv16x_gy[0]),
		// 	sensor_value_to_double(&lsm6dsv16x_gy[1]),
		// 	sensor_value_to_double(&lsm6dsv16x_gy[2])
		// );

		// printf(
		// 	"gyrox: %.8f\n", 
		// 	gyro_s_x - 2 * 0.0139 * t
		// );
		// printf(
		// 	"gyroy: %.8f\n", 
		// 	gyro_s_y + 2 * 0.0216 * t 
		// );
		// printf(
		// 	"gyroz: %.8f\n", 
		// 	gyro_s_z
		// );

		/*
		printf("LSM6DSV16X: Temperature: %.1f C\n",
		    sensor_value_to_double(&lsm6dsv16x_temp)
		);
		*/

		// printf(
		// 	"LPS22DF: Temperature: %.1f C\n", sensor_value_to_double(&lps22df_temp)
		// );
		// printf(
		// 	"LPS22DF: Pressure: %.3f kpa\n", sensor_value_to_double(&lps22df_press)
		// );

		// Complementär-Filter

		update_complementary_filter(
			acc_a_x, 
			acc_a_y, 
			acc_a_z,
			gyro_v_x, 
			gyro_v_y, 
			gyro_v_z,
			mag_a[X],
			mag_a[Y],
			mag_a[Z],
			dt
		);

		Attitude current_attitude = get_attitude();

		double accel_roll = atan2f(
			acc_a_y,
			acc_a_z
		);
		double accel_pitch = atan2f(
			-acc_a_x, 
			sqrtf(
				powf(acc_a_y, 2) + 
				powf(acc_a_z, 2)
			)
		);

		// Integrate gyroscope readings
		attitude.roll = COMPLEMENTARY_FILTER_ALPHA * (
			attitude.roll + 
			gyro_v_x * dt
		) + (1 - COMPLEMENTARY_FILTER_ALPHA) * accel_roll;
		attitude.pitch = COMPLEMENTARY_FILTER_ALPHA * (
			attitude.pitch + 
			gyro_v_y * dt) + (
				1 - COMPLEMENTARY_FILTER_ALPHA
			) * accel_pitch;
		attitude.yaw += gyro_v_z * dt;

		printf(
			"attx: %.2f\n", 
			current_attitude.roll * RAD_TO_DEG
		);
		printf(
			"atty: %.2f\n", 
			current_attitude.pitch * RAD_TO_DEG
		);
		printf(
			"attz: %.2f\n", 
			current_attitude.yaw * RAD_TO_DEG
		);

		// Extended Kalman-Filter

		// EKF_update(
		// 	&ekf, 
		// 	euler_ekf, 
		// 	acc_a_x, 
		// 	acc_a_y,
		// 	acc_a_z,
		// 	gyro_v_x,
		// 	gyro_v_y,
		// 	gyro_v_z,
		// 	mag_a[X],
		// 	mag_a[Y],
		// 	mag_a[Z],
		// 	dt
		// );
		
		// printf(
		// 	"attx:  %.2f\n", 
		// 	euler_ekf[ROLL] * 180.0 / PI
		// );
		// printf(
		// 	"atty: %.2f\n", 
		// 	euler_ekf[PITCH] * 180.0 / PI
		// );
		// printf(
		// 	"attz: %.2f\n", 
		// 	euler_ekf[YAW] * 180.0 / PI
		// );

		// Controller
		quad_a[0] = acc_a_x;
		quad_a[1] = acc_a_y;
		quad_a[2] = acc_a_z;
		quad_v[0] = acc_v_x;
		quad_v[1] = acc_v_y;
		quad_v[2] = acc_v_z;
		quad_rate[0] = gyro_v_x;
		quad_rate[1] = gyro_v_y;
		quad_rate[2] = gyro_v_z;

		// quad_a[0] = 0.0;
		// quad_a[1] = 0.0;
		// quad_a[2] = 0.0;
		// quad_v[0] = 0.0;
		// quad_v[1] = 0.0;
		// quad_v[2] = 0.0;
		// quad_rate[0] = 0.0;
		// quad_rate[1] = 0.0;
		// quad_rate[2] = 0.0;

		// Controller v_z
		v_error[Z] = v_target[Z] - quad_v[Z];
		// double thrust_z;
		
		thrust_sp[2] = vel_p_gain[2] * v_error[2] - vel_d_gain[2] * quad_a[2] + quadcopter_mass * (acc_setpoint[2] - gravitation_constant) + thr_int[2];

		double uMax = -0.4;
		double uMin = -16 * 9.18;

		// thrust_sp[2] = thrust_z;
		if (thrust_sp[2] < uMin) {
			thrust_sp[2] = uMin;
		}
		if (thrust_sp[2] > uMax) {
			thrust_sp[2] = uMax;
		}

		// Controller v_x, v_y

		// XY Velocity Control (Thrust in NE-direction)
		v_error[X] = v_target[X] - quad_v[X];
		v_error[Y] = v_target[Y] - quad_v[Y];
		thrust_sp[X] = (
			vel_p_gain[X] * v_error[X] - 
			vel_d_gain[X] * quad_a[X] + 
			quadcopter_mass * acc_setpoint[X] + 
			thr_int[X]
		);
		thrust_sp[Y] = (
			vel_p_gain[Y] * v_error[Y] - 
			vel_d_gain[Y] * quad_a[Y] + 
			quadcopter_mass * acc_setpoint[Y] + 
			thr_int[Y]
		);

		double thrust_max_xy_tilt = (
			abs(thrust_sp[2]) * tan(tiltMax)
		);
		double thrust_max_xy = sqrt(
			pow(maxThr, 2) - 
			pow(thrust_sp[Z], 2)
		);

		if (thrust_max_xy > thrust_max_xy_tilt) {
			thrust_max_xy = thrust_max_xy_tilt;
		}

		if (
			thrust_sp[X] * thrust_sp[X] + thrust_sp[Y] * thrust_sp[Y] >
			pow(thrust_max_xy, 2)) {
			
			double mag = sqrt(
				pow(thrust_sp[X], 2) +
				pow(thrust_sp[Y], 2)
			);

			thrust_sp[X] = (
				thrust_sp[X] * thrust_max_xy / mag
			);
			thrust_sp[Y] = (
				thrust_sp[Y] * thrust_max_xy / mag
			);
		}
		
		// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
		// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990

		v_error_lim[X] = v_error[X] - (
			thrust_sp[X] - thrust_sp[X]
			) * 2.0 / vel_p_gain[X];
		v_error_lim[Y] = v_error[X] - (
			thrust_sp[Y] - thrust_sp[Y]
			) * 2.0 / vel_p_gain[Y];

		double tmpnorm = sqrt(
			thrust_sp[0] * thrust_sp[0] + 
			thrust_sp[1] * thrust_sp[1] + 
			thrust_sp[2] * thrust_sp[2] 
		);

		body_z[0] = -thrust_sp[0] / tmpnorm;
		body_z[1] = -thrust_sp[1] / tmpnorm;
		body_z[2] = -thrust_sp[2] / tmpnorm;

		y_C[0] = -sin(yaw_setpoint);
		y_C[1] = cos(yaw_setpoint);
		y_C[2] = 0.0;  

		// Kreuzprodukt
		body_x[0] = y_C[1] * body_z[2] - y_C[2] * body_z[1];
		body_x[1] = y_C[2] * body_z[0] - y_C[0] * body_z[2];
		body_x[2] = y_C[0] * body_z[1] - y_C[1] * body_z[0];

		double normBodyX = sqrt(
			body_x[0] * body_x[0] + 
			body_x[1] * body_x[1] + 
			body_x[2] * body_x[2]
		);

		body_x[0] = body_x[0] / normBodyX;
		body_x[1] = body_x[1] / normBodyX;
		body_x[2] = body_x[2] / normBodyX;
			
		body_y[0] = (
			body_z[1] * body_x[2] - 
			body_z[2] * body_x[1]
		);
		body_y[1] = (
			body_z[2] * body_x[0] - 
			body_z[0] * body_x[2]
		);
		body_y[2] = (
			body_z[0] * body_x[1] - 
			body_z[1] * body_x[0]
		);

		double R_sp[3][3];

		// Desired rotation matrix
		R_sp[0][0] = body_x[0];
		R_sp[1][0] = body_x[1];
		R_sp[2][0] = body_x[2];

		R_sp[0][1] = body_y[0];
		R_sp[1][1] = body_y[1];
		R_sp[2][1] = body_y[2];

		R_sp[0][2] = body_z[0];
		R_sp[1][2] = body_z[1];
		R_sp[2][2] = body_z[2];

		// Full desired quaternion (
		//    full because it considers the desired Yaw angle
		// )
		// RotToQuat(R_sp, q_target);


		double qd_full[4];

		double tr = R_sp[0][0] + R_sp[1][1] + R_sp[2][2];
		double r, e0, e1, e2, e3;

		if (tr > R_sp[0][0] && tr > R_sp[1][1] && tr > R_sp[2][2]) {
			e0 = 0.5 * sqrt(1 + tr);
			r = 0.25 / e0;
			e1 = (R_sp[2][1] - R_sp[1][2]) * r;
			e2 = (R_sp[0][2] - R_sp[2][0]) * r;
			e3 = (R_sp[1][0] - R_sp[0][1]) * r;
		} else if (R_sp[0][0] > R_sp[1][1] && R_sp[0][0] > R_sp[2][2]) {
			e1 = 0.5 * sqrt(1 - tr + 2 * R_sp[0][0]);
			r = 0.25 / e1;
			e0 = (R_sp[2][1] - R_sp[1][2]) * r;
			e2 = (R_sp[0][1] + R_sp[1][0]) * r;
			e3 = (R_sp[0][2] + R_sp[2][0]) * r;
		} else if (R_sp[1][1] > R_sp[2][2]) {
			e2 = 0.5 * sqrt(1 - tr + 2 * R_sp[1][1]);
			r = 0.25 / e2;
			e0 = (R_sp[0][2] - R_sp[2][0]) * r;
			e1 = (R_sp[0][1] + R_sp[1][0]) * r;
			e3 = (R_sp[1][2] + R_sp[2][1]) * r;
		} else {
			e3 = 0.5 * sqrt(1 - tr + 2 * R_sp[2][2]);
			r = 0.25 / e3;
			e0 = (R_sp[1][0] - R_sp[0][1]) * r;
			e1 = (R_sp[0][2] + R_sp[2][0]) * r;
			e2 = (R_sp[1][2] + R_sp[2][1]) * r;
		}

		qd_full[0] = e0;
		qd_full[1] = e1;
		qd_full[2] = e2;
		qd_full[3] = e3;

		if (e0 < 0) {
			qd_full[0] = -qd_full[0];
			qd_full[1] = -qd_full[1];
			qd_full[2] = -qd_full[2];
			qd_full[3] = -qd_full[3];
		}

		norm = sqrt(
			qd_full[0] * qd_full[0] + 
			qd_full[1] * qd_full[1] + 
			qd_full[2] * qd_full[2] + 
			qd_full[3] * qd_full[3]
		);

		qd_full[0] = qd_full[0] / norm;
		qd_full[1] = qd_full[1] / norm;
		qd_full[2] = qd_full[2] / norm;
		qd_full[3] = qd_full[3] / norm;


		// Maybe not correct
		// qd_full[0] = q_target[0];
		// qd_full[1] = q_target[1];
		// qd_full[2] = q_target[2];
		// qd_full[3] = q_target[3];

		// Current thrust orientation e_z and desired thrust orientation e_z_d

		quad_dcm[0][0] = 1.0;
		quad_dcm[1][1] = 1.0;
		quad_dcm[2][2] = 1.0;

		e_z[0] = quad_dcm[0][2];
		e_z[1] = quad_dcm[1][2];
		e_z[2] = quad_dcm[2][2];

		double normThrustSp = sqrt(
			thrust_sp[0] * thrust_sp[0] + 
			thrust_sp[1] * thrust_sp[1] + 
			thrust_sp[2] * thrust_sp[2] 
		);

		e_z_d[0] = -thrust_sp[0] / normThrustSp;
		e_z_d[1] = -thrust_sp[1] / normThrustSp;
		e_z_d[2] = -thrust_sp[2] / normThrustSp;

		double qe_red_dot_part = (
			e_z[0] * e_z_d[0] +
			e_z[1] * e_z_d[1] + 
			e_z[2] * e_z_d[2]
		);

		e_z_norm = sqrt(
			e_z[0] * e_z[0] + 
			e_z[1] * e_z[1] + 
			e_z[2] * e_z[2]
		);

		e_z[0] = -e_z[0] / e_z_norm;
		e_z[1] = -e_z[1] / e_z_norm;
		e_z[2] = -e_z[2] / e_z_norm;

		double e_z_d_norm = sqrt(
			e_z_d[0] * e_z_d[0] + 
			e_z_d[1] * e_z_d[1] + 
			e_z_d[2] * e_z_d[2] 
		);

		e_z_d[0] = -e_z_d[0] / e_z_d_norm;
		e_z_d[1] = -e_z_d[1] / e_z_d_norm;
		e_z_d[2] = -e_z_d[2] / e_z_d_norm;

		qe_red[0] = qe_red_dot_part + sqrt(
			e_z[0] * e_z[0] +
			e_z[1] * e_z[1] +
			e_z[2] * e_z[2] +
			e_z_d[0] * e_z_d[0] +
			e_z_d[1] * e_z_d[1] +
			e_z_d[2] * e_z_d[2] 
		);
		
		qe_red[1] = (
			e_z[1] * e_z_d[2] - 
			e_z[2] * e_z_d[1]
		);
		qe_red[2] = (
			e_z[2] * e_z_d[0] - 
			e_z[0] * e_z_d[2]
		);
		qe_red[3] = (
			e_z[0] * e_z_d[1] - 
			e_z[1] * e_z_d[0]
		);

		qe_norm = sqrt(
			qe_red[0] * qe_red[0] + 
			qe_red[1] * qe_red[1] + 
			qe_red[2] * qe_red[2] + 
			qe_red[3] * qe_red[3]
		);

		qe_red[0] = qe_red[0] / qe_norm;
		qe_red[1] = qe_red[1] / qe_norm;
		qe_red[2] = qe_red[2] / qe_norm;
		qe_red[3] = qe_red[3] / qe_norm;

		// Reduced desired quaternion (reduced because it doesn't consider the desired Yaw angle)

		qd_red[0] = (
			qe_red[0] * quad_quat_state[0] - 
			qe_red[1] * quad_quat_state[1] - 
			qe_red[2] * quad_quat_state[2] - 
			qe_red[3] * quad_quat_state[3]
		);
		qd_red[1] = (
			qe_red[1] * quad_quat_state[0] + 
			qe_red[0] * quad_quat_state[1] - 
			qe_red[3] * quad_quat_state[2] + 
			qe_red[2] * quad_quat_state[3]
		);
		qd_red[2] = (
			qe_red[2] * quad_quat_state[0] + 
			qe_red[3] * quad_quat_state[1] + 
			qe_red[0] * quad_quat_state[2] - 
			qe_red[1] * quad_quat_state[3]
		);
		qd_red[3] = (
			qe_red[3] * quad_quat_state[0] - 
			qe_red[2] * quad_quat_state[1] + 
			qe_red[1] * quad_quat_state[2] + 
			qe_red[0] * quad_quat_state[3]
		);

		// Mixed desired quaternion (between reduced and full) and resulting desired quaternion qd

		qd_norm = sqrt(
			qd_red[0] * qd_red[0] + 
			qd_red[1] * qd_red[1] + 
			qd_red[2] * qd_red[2] + 
			qd_red[3] * qd_red[3]
		);

		qd_red_inverse[0] =  qd_red[0] / qd_norm;
		qd_red_inverse[1] = -qd_red[1] / qd_norm;
		qd_red_inverse[2] = -qd_red[2] / qd_norm;
		qd_red_inverse[3] = -qd_red[3] / qd_norm;

		q_mix[0] = (
			qd_red_inverse[0] * qd_full[0] - 
			qd_red_inverse[1] * qd_full[1] - 
			qd_red_inverse[2] * qd_full[2] - 
			qd_red_inverse[3] * qd_full[3]
		);

		q_mix[1] = (
			qd_red_inverse[1] * qd_full[0] + 
			qd_red_inverse[0] * qd_full[1] - 
			qd_red_inverse[3] * qd_full[2] + 
			qd_red_inverse[2] * qd_full[3]
		);
		q_mix[2] = (
			qd_red_inverse[2] * qd_full[0] + 
			qd_red_inverse[3] * qd_full[1] + 
			qd_red_inverse[0] * qd_full[2] - 
			qd_red_inverse[1] * qd_full[3]
		);
		q_mix[3] = (
			qd_red_inverse[3] * qd_full[0] - 
			qd_red_inverse[2] * qd_full[1] + 
			qd_red_inverse[1] * qd_full[2] + 
			qd_red_inverse[0] * qd_full[3]
		);

		if (q_mix[0] < -1.0) {
			q_mix[0] = -1.0;
		}
		if (q_mix[0] > 1.0) {
			q_mix[0] = 1.0;
		}
		if (q_mix[3] < -1.0) {
			q_mix[3] = -1.0;
		}
		if (q_mix[3] > 1.0) {
			q_mix[3] = 1.0;
		}
		
		qd[0] = (
			qd_red[0] * cos(yaw_w * acos(q_mix[0])) - 
			qd_red[3] * sin(yaw_w * asin(q_mix[3]))
		);
		qd[1] = (
			qd_red[1] * cos(yaw_w * acos(q_mix[0])) + 
			qd_red[2] * sin(yaw_w * asin(q_mix[3]))
		);
		qd[2] = (
			qd_red[2] * cos(yaw_w * acos(q_mix[0])) - 
			qd_red[1] * sin(yaw_w * asin(q_mix[3]))
		);
		qd[3] = (
			qd_red[3] * cos(yaw_w * acos(q_mix[0])) + 
			qd_red[0] * sin(yaw_w * asin(q_mix[3]))
		);
		
		nor = sqrt(
			pow(quad_quat_state[0], 2) + 
			pow(quad_quat_state[1], 2) + 
			pow(quad_quat_state[2], 2) + 
			pow(quad_quat_state[3], 2)
		);
		
		quad_quat_state_inverse[0] =  quad_quat_state[0] / nor;
		quad_quat_state_inverse[1] = -quad_quat_state[1] / nor;
		quad_quat_state_inverse[2] = -quad_quat_state[2] / nor;
		quad_quat_state_inverse[3] = -quad_quat_state[3] / nor;

		// Resulting error quaternion

		quadternion_error[0] = (
			quad_quat_state_inverse[0] * qd[0] - 
			quad_quat_state_inverse[1] * qd[1] - 
			quad_quat_state_inverse[2] * qd[2] - 
			quad_quat_state_inverse[3] * qd[3]
		);
		quadternion_error[1] = (
			quad_quat_state_inverse[1] * qd[0] + 
			quad_quat_state_inverse[0] * qd[1] - 
			quad_quat_state_inverse[3] * qd[2] + 
			quad_quat_state_inverse[2] * qd[3]
		);
		quadternion_error[2] = (
			quad_quat_state_inverse[2] * qd[0] + 
			quad_quat_state_inverse[3] * qd[1] + 
			quad_quat_state_inverse[0] * qd[2] - 
			quad_quat_state_inverse[1] * qd[3]
		);
		quadternion_error[3] = (
			quad_quat_state_inverse[3] * qd[0] - 
			quad_quat_state_inverse[2] * qd[1] + 
			quad_quat_state_inverse[1] * qd[2] + 
			quad_quat_state_inverse[0] * qd[3]
		);

		// Create rate setpoint from quaternion error

		if (quadternion_error[0] > 0) {
			rate_setpoint[0] = (
				2.0 * quadternion_error[1] * attitute_p_gain[0]
			);
			rate_setpoint[1] = (
				2.0 * quadternion_error[2] * attitute_p_gain[1]
			);
			rate_setpoint[2] = (
				2.0 * quadternion_error[3] * attitute_p_gain[2]
			);
		} else {
			rate_setpoint[0] = (
				-2.0 * quadternion_error[1] * attitute_p_gain[0]
			);
			rate_setpoint[1] = (
				-2.0 * quadternion_error[2] * attitute_p_gain[1]
			);
			rate_setpoint[2] = (
				-2.0 * quadternion_error[3] * attitute_p_gain[2]
			);
		}

		// Rate Control
		for (int i = 0; i < 3; i++) {
			rate_error[i] = rate_setpoint[i] - quad_rate[i];
			rateCtrl[i] = (
				rate_p_gain[i] * rate_error[i] - 
				rate_d_gain[i] * omega_dot[i]
			);
		}

		// Mixer

		thrust_sp_norm = sqrt(
			pow(thrust_sp[0], 2) +
			pow(thrust_sp[1], 2) +
			pow(thrust_sp[2], 2)
		);

		w_cmd[0] = (
			mixerFM[0][0] * thrust_sp_norm + 
			mixerFM[0][1] * rateCtrl[X] +
			mixerFM[0][2] * rateCtrl[Y] +
			mixerFM[0][3] * rateCtrl[Z]
		);  
		w_cmd[1] = (
			mixerFM[1][0] * thrust_sp_norm + 
			mixerFM[1][1] * rateCtrl[X] +
			mixerFM[1][2] * rateCtrl[Y] +
			mixerFM[1][3] * rateCtrl[Z]
		);  
		w_cmd[2] = (
			mixerFM[2][0] * thrust_sp_norm + 
			mixerFM[2][1] * rateCtrl[X] +
			mixerFM[2][2] * rateCtrl[Y] +
			mixerFM[2][3] * rateCtrl[Z]
		);  
		w_cmd[3] = (
			mixerFM[3][0] * thrust_sp_norm + 
			mixerFM[3][1] * rateCtrl[X] +
			mixerFM[3][2] * rateCtrl[Y] +
			mixerFM[3][3] * rateCtrl[Z]
		);  

		// printf(
		// 	"test: %.8f\n", mixerFM[0][2] * rateCtrl[Y]
		// );

		for (int i = 0; i < 4; i++) {
			w_cmd_clipped[i] = sqrt(w_cmd[i]);
			if (w_cmd_clipped[i] < minWMotor) {
				w_cmd_clipped[i] = minWMotor;
			};
			if (w_cmd_clipped[i] > maxWMotor) {
				w_cmd_clipped[i] = maxWMotor;
			};
		};

		printf(
			"cmd1: %.2f\n", w_cmd_clipped[0]
		);
		printf(
			"cmd2: %.2f\n", w_cmd_clipped[1]
		);
		printf(
			"cmd3: %.2f\n", w_cmd_clipped[2]
		);
		printf(
			"cmd4: %.2f\n", w_cmd_clipped[3]
		);

		// PWM Motor control

        pwm_set(
			pwm_dev, 
			PWM_CHANNEL, 
			PWM_USEC(20000), 
			PWM_USEC(10000), 
			0
		);


#if defined(CONFIG_LIS2MDL_TRIGGER)
		printk("%d: lis2mdl trig %d\n", cnt, lis2mdl_trig_cnt);
#endif

#ifdef CONFIG_LSM6DSO16IS_TRIGGER
		printk("%d: lsm6dso16is acc trig %d\n", cnt, lsm6dso16is_acc_trig_cnt);
#endif

#ifdef CONFIG_LSM6DSV16X_TRIGGER
		printk("%d: lsm6dsv16x acc trig %d\n", cnt, lsm6dsv16x_acc_trig_cnt);
#endif
#ifdef CONFIG_LPS2XDF_TRIGGER
		printk("%d: lps22df trig %d\n", cnt, lps22df_trig_cnt);
#endif

		t += delta_t;

		// printf("t: %.2f\n", t);

		cnt++;
		k_sleep(K_MSEC(1));
	}
}
