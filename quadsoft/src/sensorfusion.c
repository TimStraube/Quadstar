#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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


void EKF_update(
	ekf_t* ekf, 
	double euler[3], 
	double ax, 
	double ay, 
	double az, 
	double p, 
	double q, 
	double r,
	double mx, 
	double my, 
	double mz, 
	double dt) {
	
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
double v_x_raw = 0;
double v_y_raw = 0;
double v_z_raw = 0;
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

double g_estimate[3];
double g_norm;


double acc_a_z_old;
double v_z_raw_old;

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
    double accel_roll = atan2f(
		acc_y, 
		acc_z
	);
    double accel_pitch = atan2f(
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
    double mag_yaw = atan2f(-mag_y_comp, mag_x_comp);

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


struct euler_angles {
	double roll;
	double pitch;
	double yaw;
};


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
	sensor_trigger_set(
		lsm6dsv16x, 
		&trig, 
		lsm6dsv16x_acc_trig_handler
	);
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

	g_estimate[X] = 0.0;
	g_estimate[Y] = 0.0;
	g_estimate[Z] = -0.981;

	ekf_t ekf;
	double euler_ekf[3];

	mag_a[X] = 1.0;
	mag_a[Y] = 1.0;
	mag_a[Z] = -9.81;


	EKF_init(
		&ekf, 
		mag_a[X], 
		mag_a[Y], 
		mag_a[Z], 
		0.1, 
		1, 
		100
	);

	while (1) {

#ifndef CONFIG_LIS2MDL_TRIGGER
		if (sensor_sample_fetch(lis2mdl) < 0) {
			printf(
				"LIS2MDL Magn Sensor sample update error\n"
			);
			return 0;
		}
#endif
#ifndef CONFIG_LSM6DSO16IS_TRIGGER
		if (sensor_sample_fetch(lsm6dso16is) < 0) {
			printf(
				"LSM6DSO16IS Sensor sample update error\n"
			);
			return 0;
		}
#endif
#ifndef CONFIG_LSM6DSV16X_TRIGGER
		if (sensor_sample_fetch(lsm6dsv16x) < 0) {
			printf(
				"LSM6DSV16X Sensor sample update error\n"
			);
			return 0;
		}
#endif
#ifndef CONFIG_LPS2XDF_TRIGGER
		if (sensor_sample_fetch(lps22df) < 0) {
			printf(
				"LPS22DF pressure sample update error\n"
			);
			return 0;
		}
#endif

		sensor_channel_get(
			lis2mdl, 
			SENSOR_CHAN_MAGN_XYZ, 
			lis2mdl_magn
		);
		sensor_channel_get(
			lis2mdl, 
			SENSOR_CHAN_DIE_TEMP, 
			&lis2mdl_temp
		);
		sensor_channel_get(
			lsm6dso16is, 
			SENSOR_CHAN_ACCEL_XYZ, 
			lsm6dso16is_xl
		);
		sensor_channel_get(
			lsm6dso16is, 
			SENSOR_CHAN_GYRO_XYZ, 
			lsm6dso16is_gy
		);
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

		// Accelometer
		// acc_a_x = (
		// 	sensor_value_to_double(&lsm6dsv16x_xl[X]) + 
		// 	sensor_value_to_double(&lsm6dso16is_xl[X])
		// ) / 2;
		// acc_a_y = (
		// 	sensor_value_to_double(&lsm6dsv16x_xl[Y]) +
		// 	sensor_value_to_double(&lsm6dso16is_xl[Y])
		// ) / 2;
		// acc_a_z = (
		// 	sensor_value_to_double(&lsm6dsv16x_xl[Z]) + 
		// 	sensor_value_to_double(&lsm6dso16is_xl[Z])
		// ) / 2;

		// Accelometer
		acc_a_x = sensor_value_to_double(
			&lsm6dsv16x_xl[X]
		);
		acc_a_y = sensor_value_to_double(
			&lsm6dsv16x_xl[Y]
		);
		acc_a_z = sensor_value_to_double(
			&lsm6dsv16x_xl[Z]
		);

		// float alpha = 0.8;

		// gravity[0] = alpha * gravity[0] + (1 - alpha) * event.values[0];
		// gravity[1] = alpha * gravity[1] + (1 - alpha) * event.values[1];
		// gravity[2] = alpha * gravity[2] + (1 - alpha) * event.values[2];

		// linear_acceleration[0] = event.values[0] - gravity[0];
		// linear_acceleration[1] = event.values[1] - gravity[1];
		// linear_acceleration[2] = event.values[2] - gravity[2];


		// acc_a_z = 0.5 * acc_a_z + 0.5 * acc_a_z_old;

		// acc_a_z_old = acc_a_z;


		acc_a_x = acc_a_x + g_estimate[X];
		acc_a_y = acc_a_y + g_estimate[Y];
		acc_a_z = acc_a_z + g_estimate[Z];

		if (abs(acc_a_x) < 0.0001) {
			acc_a_x = 0.0;
		} 
		if (abs(acc_a_y) < 0.0001) {
			acc_a_y = 0.0;
		} 
		if (abs(acc_a_z) < 0.0001) {
			acc_a_z = 0.0;
		} 

		v_x_raw += dt * acc_a_x;
		v_y_raw += dt * acc_a_y;
		v_z_raw += dt * acc_a_z;

		// v_z_raw = 0.8 * v_z_raw + 0.2 * v_z_raw_old;

		// v_z_raw_old = v_z_raw;

		// acc_s_x = acc_s_x + dt * (
		// 	v_x_raw
		// );
		// acc_s_y = acc_s_y + dt * (
		// 	v_y_raw
		// );
		// acc_s_z = acc_s_z + dt * (
		// 	v_z_raw
		// );


		// printf("accx: %.4f\n", acc_a_x);
		// printf("accy: %.4f\n", acc_a_y);
		// printf("accz: %.4f\n", acc_a_z);


		// printf("velx: %.4f\n", v_x_raw);
		// printf("vely: %.4f\n", v_y_raw);
		// printf("velz: %.4f\n", v_z_raw);


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

		// Komplementärfilter 1

		// update_complementary_filter(
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

		// Attitude attitude = get_attitude();

		// Komplementärfilter 2

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
			"attx: %.4f\n", 
			attitude.roll * RAD_TO_DEG
		);
		printf(
			"atty: %.4f\n", 
			attitude.pitch * RAD_TO_DEG
		);
		printf(
			"attz: %.4f\n", 
			attitude.yaw * RAD_TO_DEG
		);

		g_estimate[X] = sin(attitude.roll);
		g_estimate[Y] = sin(attitude.pitch);
		g_estimate[Z] = -1.0;

		g_norm = sqrt(
			g_estimate[X] * g_estimate[X] +
			g_estimate[Y] * g_estimate[Y] + 
			g_estimate[Z] * g_estimate[Z]
		);

		g_estimate[X] = 9.81 * (g_estimate[X] / g_norm);
		g_estimate[Y] = 9.81 * (g_estimate[Y] / g_norm);
		g_estimate[Z] = 9.81 * (g_estimate[Z] / g_norm);

		// printf(
		// 	"gestimatex: %.4f\n", 
		// 	g_estimate[X]
		// );
		// printf(
		// 	"gestimatey: %.4f\n", 
		// 	g_estimate[Y]
		// );
		// printf(
		// 	"gestimatez: %.4f\n", 
		// 	g_estimate[Z]
		// );

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

		t += dt;

		cnt++;
		k_sleep(K_MSEC(0.01));
	}
}
