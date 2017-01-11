/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

const double deg2rad = M_PI / 180;
const double rad2deg = 180 / M_PI;
const float deg2radf = M_PI_F / 180;
const float rad2degf = 180 / M_PI_F;

/**
 * Note that structs are used instead of enums
 * to allow use in arrays without casting
 * and to keep size small
 */

/**
 * State enum
 */
struct X {
	static const uint8_t q_nb_0 = 0;
	static const uint8_t q_nb_1 = 1;
	static const uint8_t q_nb_2 = 2;
	static const uint8_t q_nb_3 = 3;
	static const uint8_t vel_N = 4;
	static const uint8_t vel_E = 5;
	static const uint8_t vel_D = 6;
	static const uint8_t gyro_bias_bX = 7;
	static const uint8_t gyro_bias_bY = 8;
	static const uint8_t gyro_bias_bZ = 9;
	static const uint8_t accel_scale = 10;
	static const uint8_t pos_N = 11;
	static const uint8_t pos_E = 12;
	static const uint8_t asl = 13;
	static const uint8_t terrain_asl = 14;
	static const uint8_t baro_bias = 15;
	static const uint8_t wind_N = 16;
	static const uint8_t wind_E = 17;
	static const uint8_t wind_D = 18;
	static const uint8_t n = 19;
};

/**
 * Error state enum
 * used for linearization
 *
 * Note gyro bias in navigation frame
 */
struct Xe {
	static const uint8_t rot_N = 0;
	static const uint8_t rot_E = 1;
	static const uint8_t rot_D = 2;
	static const uint8_t vel_N = 3;
	static const uint8_t vel_E = 4;
	static const uint8_t vel_D = 5;
	static const uint8_t gyro_bias_N = 6;
	static const uint8_t gyro_bias_E = 7;
	static const uint8_t gyro_bias_D = 8;
	static const uint8_t accel_scale = 9;
	static const uint8_t pos_N = 10;
	static const uint8_t pos_E = 11;
	static const uint8_t asl = 12;
	static const uint8_t terrain_asl = 13;
	static const uint8_t baro_bias = 14;
	static const uint8_t wind_N = 15;
	static const uint8_t wind_E = 16;
	static const uint8_t wind_D = 17;
	static const uint8_t n = 18;
};

/**
 * Input enum
 */
struct U {
	static const uint8_t omega_nb_bX = 0;
	static const uint8_t omega_nb_bY = 1;
	static const uint8_t omega_nb_bZ = 2;
	static const uint8_t accel_bX = 3;
	static const uint8_t accel_bY = 4;
	static const uint8_t accel_bZ = 5;
	static const uint8_t n = 6;
};

/**
 * Accel measurement enum
 */
struct Y_accel {
	static const uint8_t accel_bX = 0;
	static const uint8_t accel_bY = 1;
	static const uint8_t accel_bZ = 2;
	static const uint8_t n = 3;
};

/**
 * Land enum
 */
struct Y_land {
	static const uint8_t vel_N = 0;
	static const uint8_t vel_E = 1;
	static const uint8_t agl = 2;
	static const uint8_t n = 3;
};

/**
 * GPS measurement
 */
struct Y_gps {
	static const uint8_t pos_N = 0;
	static const uint8_t pos_E = 1;
	static const uint8_t asl = 2;
	static const uint8_t vel_N = 3;
	static const uint8_t vel_E = 4;
	static const uint8_t vel_D = 5;
	static const uint8_t n = 6;
};

/**
 * Baro measurement
 */
struct Y_baro {
	static const uint8_t asl = 0;
	static const uint8_t n = 1;
};

/**
 * Magnetometer measurement
 *
 * The filter treats the error
 * in the navigation frame
 * (north, east, down) even though the
 * field is measured in the body
 * frame.
 */
struct Y_mag {
	static const uint8_t mag_N = 0;
	static const uint8_t mag_E = 1;
	//static const uint8_t mag_D = 2;
	static const uint8_t n = 2;
};

/**
 * Airspeed measurement
 */
struct Y_airspeed {
	static const uint8_t airspeed = 0;
	static const uint8_t n = 1;
};

/**
 * Optical flow measurement
 */
struct Y_flow {
	static const uint8_t flowX = 0;
	static const uint8_t flowY = 1;
	static const uint8_t n = 2;
};

/**
 * Distance down measurement
 */
struct Y_distance_down {
	static const uint8_t d = 0;
	static const uint8_t n = 1;
};

static const float BETA_TABLE[] = {
	0,
	8.82050518214,
	12.094592431,
	13.9876612368,
	16.0875642296,
	17.8797700658,
	19.6465647819,
	21.3802576894,
	23.0806434845,
	24.6673803845,
	26.1487953661,
	27.6350821245,
	29.6565383703,
	31.2211113844,
	32.7673547211,
	34.2967756977,
	35.6906782236,
	37.0724753352,
	38.4549693067,
	39.836592699,
};

// TODO these should all be rosparams eventually

// guesses
static const float wind_correlation_time = 1000; // s
static const float flow_sigma_rw = 1e-3f; // m/s / sqrt(s)

static const float gps_xy_sigma_rw = 5e-1f; // m / sqrt(s)
static const float gps_vxy_sigma_rw = 5e-2f; // (m/s) / sqrt(s)
static const float gps_z_sigma_rw = 2e0f; // m / sqrt(s)
static const float gps_vz_sigma_rw = 1e0f; // (m/s) / sqrt(s)

static const float process_noise_sigma_xy = 0; // (m) / sqrt(s)
static const float process_noise_sigma_vxy = 0; // (m/s) / sqrt(s)
static const float process_noise_sigma_z = 0; // (m) / sqrt(s)
static const float process_noise_sigma_vz = 0; // (m/s) / sqrt(s)
static const float process_noise_sigma_rot = 0; // (rad) / sqrt(s)

//-----------------------------------------------------------------

#ifdef CONFIG_ARCH_BOARD_SITL

// SITL
static const float magDeclDeg = -0.745;
// zurich decl output from sitl code is -0.745 deg, should be 2.05
// possibly outdated mag model (2014)
static const float magInclDeg = 63.3f;

static const float gyro_sigma_rw = 4.6e-4f; // rad / sqrt(s)
static const float gyro_sigma_rrw = 4.2e-5f; // rad/s / sqrt(s)
static const float gyro_correlation_time = 500.0; // s

static const float accel_sigma_rw = 5.3e-3f; // rad / sqrt(s)
static const float accel_sigma_rrw = 5.0e-3; // rad/s / sqrt(s)
static const float accel_correlation_time = 300.0; // s

static const float baro_sigma_rw = 1.2e-1; // m / sqrt(s)
static const float baro_sigma_rrw = 0; // (m/s) / sqrt(s)
static const float baro_correlation_time = 1e3; // s

static const float mag_sigma_rw = 4.7e-3f; // rad / sqrt(s)
static const float mag_sigma_rrw = 0; // (ga/s) / sqrt(s)
static const float mag_correlation_time = 500; // s

static const float land_sigma_vxy = 1e-1f; // m/s
static const float land_sigma_agl = 1e-1; // m

#else

//-----------------------------------------------------------------
// pixhawk

static const float magDeclDeg = -4.7f;
static const float magInclDeg = 67.4f;

static const float gyro_sigma_rw = 9.8e-5f; // rad / sqrt(s)
static const float gyro_sigma_rrw = 1.2e-5f; // (rad/s) / sqrt(s)
static const float gyro_correlation_time = 1e3f; // s

static const float accel_sigma_rw = 2.4e-3f; // (m/s^2) / sqrt(s)
static const float accel_sigma_rrw = 2.0e-3f; // (m/s^2/s)) / sqrt(s)
static const float accel_correlation_time = 1e3f; // s

static const float baro_sigma_rw = 5.68e-2f; // m / sqrt(s)
static const float baro_sigma_rrw = 3.80e-2f; // (m/s) / sqrt(s)
static const float baro_correlation_time = 13e3f; // s

static const float mag_sigma_rw = 1.67e-3f; // ga / sqrt(s)
static const float mag_sigma_rrw = 5.46e-5f; // (ga/s) / sqrt(s)
static const float mag_correlation_time = 1e3f; // s

#endif