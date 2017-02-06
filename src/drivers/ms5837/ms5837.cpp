/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file ms5837.cpp
 * Driver for the ms5837 and MS6507 barometric pressure sensor connected via I2C or SPI.
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>
#include <board_config.h>

#include <drivers/device/device.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <uORB/topics/sensor_depth.h>
#include <uORB/topics/sensor_depth_calibration.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <platforms/px4_getopt.h>

#include "ms5837.h"

/** set current specific water density */
#define BAROIOCSWATERDENSITY	_BAROIOC(3)
/** get current specific water density */
#define BAROIOCGWATERDENSITY	_BAROIOC(4)



enum MS5837_BUS {
	MS5837_BUS_ALL = 0,
	MS5837_BUS_I2C_INTERNAL,
	MS5837_BUS_I2C_EXTERNAL
};

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/*
 * ms5837/MS5607 internal constants and data structures.
 */

/* internal conversion time: 9.17 ms, so should not be read at rates higher than 100 Hz */
#define MS5837_CONVERSION_INTERVAL	25000	/* microseconds */
#define MS5837_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */
#define MS5837_BARO_DEVICE_PATH_EXT	"/dev/ms5837_ext"
#define MS5837_BARO_DEVICE_PATH_INT	"/dev/ms5837_int"

class MS5837 : public device::CDev
{
public:
	MS5837(device::Device *interface, ms5837::prom_u &prom_buf, const char *path);
	~MS5837();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	Device			*_interface;

	ms5837::prom_s		_prom;

	struct work_s		_work;
	unsigned		_measure_ticks;

	ringbuffer::RingBuffer	*_reports;
	bool			_collect_phase;
	unsigned		_measure_phase;

	/* intermediate temperature values per ms5837/MS5607 datasheet */
	int32_t			_TEMP;
	int64_t			_OFF;
	int64_t			_SENS;
	float			_P;
	float			_T;

	/* altitude conversion calibration */
	unsigned		_msl_pressure;	/* in Pa */
	float			_water_density;

	orb_advert_t		_depth_topic;
	int			_orb_class_instance;
	int			_class_instance;

	int 				  _calibration_sub;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	/**
	 * Initialize the automatic measurement state machine and start it.
	 *
	 * @param delay_ticks the number of queue ticks before executing the next cycle
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start_cycle(unsigned delay_ticks = 1);

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop_cycle();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external() { return (_orb_class_instance == 0); /* XXX put this into the interface class */ }

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Issue a measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	virtual int		measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	virtual int		collect();
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ms5837_main(int argc, char *argv[]);

MS5837::MS5837(device::Device *interface, ms5837::prom_u &prom_buf, const char *path):
	CDev("MS5837", path),
	_interface(interface),
	_prom(prom_buf.s),
	_measure_ticks(0),
	_reports(nullptr),
	_collect_phase(false),
	_measure_phase(0),
	_TEMP(0),
	_OFF(0),
	_SENS(0),
	_msl_pressure(101325),
	_water_density(1.000),
	_depth_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	//_calibration_sub(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "ms5837_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "ms5837_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "ms5837_com_err")),
	_buffer_overflows(perf_alloc(PC_COUNT, "ms5837_buf_of"))
{
	// work_cancel in stop_cycle called from the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

MS5837::~MS5837()
{
	/* make sure we are truly inactive */
	stop_cycle();

	if (_class_instance != -1) {
		unregister_class_devname(get_devname(), _class_instance);
	}

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);

	delete _interface;
}

int
MS5837::init()
{
	int ret;

	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("CDev init failed");
		goto out;
	}

	//_calibration_sub = orb_subscribe(ORB_ID(sensor_depth_calibration));

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_baro_s));

	if (_reports == nullptr) {
		DEVICE_DEBUG("can't get memory for reports");
		ret = -ENOMEM;
		goto out;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(BARO_BASE_DEVICE_PATH);

	struct sensor_depth_s brp;
	/* do a first measurement cycle to populate reports with valid data */
	_measure_phase = 0;
	_reports->flush();

	/* this do..while is goto without goto */
	do {
		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(MS5837_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(MS5837_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		_reports->get(&brp);

		ret = OK;

		_depth_topic = orb_advertise_multi(ORB_ID(sensor_depth), &brp,
						  &_orb_class_instance, (is_external()) ? ORB_PRIO_HIGH : ORB_PRIO_DEFAULT);


		if (_depth_topic == nullptr) {
			warnx("failed to create sensor_baro publication");
		}

	} while (0);

out:
	return ret;
}

ssize_t
MS5837::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct sensor_depth_s);
	struct sensor_depth_s *brp = reinterpret_cast<struct sensor_depth_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(brp)) {
				ret += sizeof(*brp);
				brp++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_measure_phase = 0;
		_reports->flush();

		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(MS5837_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(MS5837_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(brp)) {
			ret = sizeof(*brp);
		}

	} while (0);

	return ret;
}

int
MS5837::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop_cycle();
				_measure_ticks = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(MS5837_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start_cycle();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(MS5837_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start_cycle();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);
			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/*
		 * Since we are initialized, we do not need to do anything, since the
		 * PROM is correctly read and the part does not need to be configured.
		 */
		return OK;

	case BAROIOCSMSLPRESSURE:

		/* range-check for sanity */
		if ((arg < 80000) || (arg > 120000)) {
			return -EINVAL;
		}

		_msl_pressure = arg;
		return OK;

	case BAROIOCGMSLPRESSURE:
		return _msl_pressure;

	case BAROIOCSWATERDENSITY:
		/* range-check for sanity */
		if ((arg < 5000) || (arg > 12000)) {
			return -EINVAL;
		}

		_water_density = (float)(arg/10000.0f);
		return OK;

	case BAROIOCGWATERDENSITY:
		return (int)(_water_density*10000);

	default:
		break;
	}

	/* give it to the bus-specific superclass */
	// return bus_ioctl(filp, cmd, arg);
	return CDev::ioctl(filp, cmd, arg);
}

void
MS5837::start_cycle(unsigned delay_ticks)
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&MS5837::cycle_trampoline, this, delay_ticks);
}

void
MS5837::stop_cycle()
{
	work_cancel(HPWORK, &_work);
}

void
MS5837::cycle_trampoline(void *arg)
{
	MS5837 *dev = reinterpret_cast<MS5837 *>(arg);

	dev->cycle();
}

void
MS5837::cycle()
{
	int ret;
	unsigned dummy;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (ret != OK) {
			if (ret == -6) {
				/*
				 * The MS5837 seems to regularly fail to respond to
				 * its address; this happens often enough that we'd rather not
				 * spam the console with a message for this.
				 */
			} else {
				//DEVICE_LOG("collection error %d", ret);
			}

			/* issue a reset command to the sensor */
			_interface->ioctl(IOCTL_RESET, dummy);
			/* reset the collection state machine and try again - we need
			 * to wait 2.8 ms after issuing the sensor reset command
			 * according to the MS5837 datasheet
			 */
			start_cycle(USEC2TICK(2800));
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 * Don't inject one after temperature measurements, so we can keep
		 * doing pressure measurements at something close to the desired rate.
		 */
		if ((_measure_phase != 0) &&
		    (_measure_ticks > USEC2TICK(MS5837_CONVERSION_INTERVAL))) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&MS5837::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(MS5837_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	ret = measure();

	if (ret != OK) {
		/* issue a reset command to the sensor */
		_interface->ioctl(IOCTL_RESET, dummy);
		/* reset the collection state machine and try again */
		start_cycle();
		return;
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&MS5837::cycle_trampoline,
		   this,
		   USEC2TICK(MS5837_CONVERSION_INTERVAL));
}

int
MS5837::measure()
{
	int ret;

	perf_begin(_measure_perf);

	/*
	 * In phase zero, request temperature; in other phases, request pressure.
	 */
	unsigned addr = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;

	/*
	 * Send the command to begin measuring.
	 */
	ret = _interface->ioctl(IOCTL_MEASURE, addr);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	perf_end(_measure_perf);

	return ret;
}

int
MS5837::collect()
{
	int ret;
	uint32_t raw;

	perf_begin(_sample_perf);

	struct sensor_depth_s report;
	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
	//report.error_count = perf_event_count(_comms_errors); //FIXME: add error count to depth message?

	/* read the most recent measurement - read offset/size are hardcoded in the interface */
	ret = _interface->read(0, (void *)&raw, 0);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	/* handle a measurement */
	if (_measure_phase == 0) {

		/* check if calibration msg has arrived */
		/* Not working
		bool updated;
		orb_check(_calibration_sub, &updated);

		if(updated)
		{
			struct sensor_depth_calibration_s _calib;
			orb_copy(ORB_ID(sensor_depth_calibration), _calibration_sub, &_calib);
			if(_calib.water_density > 0.8f)
				_water_density = _calib.water_density;
			if(_calib.cal_pressure > 80000)
				_msl_pressure = _calib.cal_pressure;
			if(_calib.cal_depth > 0)
			{
				_msl_pressure = (_P/10.0f) - (_water_density * 9.80665f * _calib.cal_depth);
			}
		}
		*/

		/* temperature offset (in ADC units) */
		int32_t dT = (int32_t)raw - (int32_t)(_prom.c5_reference_temp << 8);
		

		/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
		//int64_t TEMPSENS = ((int64_t)_prom.c5_reference_temp >> 23);
		//_TEMP = 2000 + (int32_t)(((int64_t)dT )*TEMPSENS);
		
		_TEMP = 2000 + (int32_t)(((int64_t)_prom.c5_reference_temp * ((int64_t)dT )) >> 23);

		/* base sensor scale/offset values */

		/* Perform MS5837 Calculation */

		_OFF  = ((int64_t)_prom.c2_pressure_offset << 16) + (((int64_t)_prom.c4_temp_coeff_pres_offset * dT) >> 7);
		_SENS = ((int64_t)_prom.c1_pressure_sens << 15) + (((int64_t)_prom.c3_temp_coeff_pres_sens * dT) >> 8);

		/* MS5837 second order temperature compensation */
		int32_t Ti 	= 0;
		int64_t OFFi 	= 0;
		int64_t SENSi 	= 0;

		if (_TEMP < 2000) 
		{
			// Low temperature (less than 20C)
			Ti 	  = (3 * ((int64_t)POW2(dT) >> 33));

			int64_t f = POW2((int64_t)_TEMP - 2000);
			OFFi 	  = 3 * f >> 1;
			SENSi 	  = 5 * f >> 3;

			if (_TEMP < -1500) {

				int64_t f2 = POW2(_TEMP + 1500);
				OFFi 	+= 7 * f2;
				SENSi 	+= 4 * f2;
			}

			
		}
		else 
		{
			// High Temperature
			Ti 	= (2 * ((int64_t)POW2(dT) >> 37));
			int64_t f = POW2((int64_t)_TEMP - 2000);
			OFFi 	  = 3 * f >> 4;
		}

		
		_OFF  -= OFFi;
		_SENS -= SENSi;
		_TEMP -= Ti;

	} 
	else {

		/* pressure calculation, result in Pa */
		int32_t P = (((raw * _SENS) >> 21) - _OFF) >> 13; // in [millibar*10]
		_P = P * 0.1f; // millibar
		_T = _TEMP * 0.01f;

		/* generate a new report */
		report.temperature   = _TEMP / 100.0f;
		report.abs_pressure  = P / 10.0f;		/* convert to millibar */
		report.cal_pressure  = (float)(_msl_pressure / 100.0f); /* convert to millibar */
		report.water_density = _water_density;

		/* return device ID */
		//report.device_id = _device_id.devid; //FIXME: add device id to depth message?


		/* current pressure at MSL in kPa */
		float p1 = _msl_pressure / 1000.0;
		
		/* measured pressure in kPa */
		float p = P / 100.0;
		float g   = 9.80665; 

		report.diff_pressure = (p-p1);
		report.depth 	     = (p-p1)/(_water_density*g);

		/* publish it */
		if (!(_pub_blocked) && _depth_topic != nullptr) {
			/* publish it */
			orb_publish(ORB_ID(sensor_depth), _depth_topic, &report);
		}

		if (_reports->force(&report)) {
			perf_count(_buffer_overflows);
		}

		/* notify anyone waiting for data */
		poll_notify(POLLIN);
	}

	/* update the measurement state machine */
	INCREMENT(_measure_phase, MS5837_MEASUREMENT_RATIO + 1);

	perf_end(_sample_perf);

	return OK;
}

void
MS5837::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
	printf("TEMP:           %d\n", _TEMP);
	printf("SENS:           %lld\n", _SENS);
	printf("OFF:            %lld\n", _OFF);
	printf("P:              %.3f\n", (double)_P);
	printf("T:              %.3f\n", (double)_T);
	printf("MSL pressure:   %10.4f\n", (double)(_msl_pressure / 100.f));
	printf("Water density:  %10.4f\n", (double)(_water_density ));

	printf("crc_and_factory_setup     %u\n", _prom.crc_and_factory_setup);
	printf("c1_pressure_sens          %u\n", _prom.c1_pressure_sens);
	printf("c2_pressure_offset        %u\n", _prom.c2_pressure_offset);
	printf("c3_temp_coeff_pres_sens   %u\n", _prom.c3_temp_coeff_pres_sens);
	printf("c4_temp_coeff_pres_offset %u\n", _prom.c4_temp_coeff_pres_offset);
	printf("c5_reference_temp         %u\n", _prom.c5_reference_temp);
	printf("c6_temp_coeff_temp        %u\n", _prom.c6_temp_coeff_temp);
}

/**
 * Local functions in support of the shell command.
 */
namespace ms5837
{

/*
  list of supported bus configurations
 */
struct ms5837_bus_option {
	enum MS5837_BUS busid;
	const char *devpath;
	ms5837_constructor interface_constructor;
	uint8_t busnum;
	MS5837 *dev;
} bus_options[] = {

#ifdef PX4_I2C_BUS_ONBOARD
	{ MS5837_BUS_I2C_INTERNAL, "/dev/ms5837_int", &ms5837_i2c_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION
	{ MS5837_BUS_I2C_EXTERNAL, "/dev/ms5837_ext", &ms5837_i2c_interface, PX4_I2C_BUS_EXPANSION, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

bool	start_bus(struct ms5837_bus_option &bus);
struct ms5837_bus_option &find_bus(enum MS5837_BUS busid);
void	start(enum MS5837_BUS busid);
void	test(enum MS5837_BUS busid);
void	reset(enum MS5837_BUS busid);
void	info();
void	calibrate(unsigned altitude, enum MS5837_BUS busid);
void	usage();

/**
 * MS5837 crc4 cribbed from the datasheet
 */
bool
crc4(uint16_t *n_prom)
{
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[0];

	/* remove CRC byte */
	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for (uint8_t i = 0; i < 16; i++) {
		/* uneven bytes */
		if (i%2 == 1) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);

		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}

	}


	/* final 4 bit remainder is CRC value */
	n_rem = ((n_rem >> 12) & 0x000F);
	//n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (crc_read >> 12) == (n_rem ^ 0x00);
}


/**
 * Start the driver.
 */
bool
start_bus(struct ms5837_bus_option &bus)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
	}

	prom_u prom_buf;
	device::Device *interface = bus.interface_constructor(prom_buf, bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new MS5837(interface, prom_buf, bus.devpath);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = NULL;
		return false;
	}

	int fd = open(bus.devpath, O_RDONLY);

	/* set the poll rate to default, starts automatic data collection */
	if (fd == -1) {
		errx(1, "can't open baro device");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		errx(1, "failed setting default poll rate");
	}

	close(fd);
	return true;
}


/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(enum MS5837_BUS busid)
{
	uint8_t i;
	bool started = false;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == MS5837_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != MS5837_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started = started | start_bus(bus_options[i]);
	}

	if (!started) {
		exit(1);
	}

	// one or more drivers started OK
	exit(0);
}


/**
 * find a bus structure for a busid
 */
struct ms5837_bus_option &find_bus(enum MS5837_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == MS5837_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(enum MS5837_BUS busid)
{
	//FIXME: Test needs to be adapted to depth msg
	struct ms5837_bus_option &bus = find_bus(busid);
	struct sensor_depth_s report;
	ssize_t sz;
	int ret;

	int fd;

	fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		err(1, "open failed (try 'ms5837 start' if the driver is not running)");
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("pressure:    %10.4f", (double)report.abs_pressure);
	warnx("depth:       %11.4f", (double)report.depth);
	warnx("temperature: %8.4f", (double)report.temperature);
	warnx("time:        %lld", report.timestamp);

	/* set the queue depth to 10 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10)) {
		errx(1, "failed to set queue depth");
	}

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("pressure:    %10.4f", (double)report.abs_pressure);
		warnx("depth:       %11.4f", (double)report.depth);
		warnx("temperature: %8.4f", (double)report.temperature);
		warnx("time:        %lld", report.timestamp);
	}

	close(fd);
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset(enum MS5837_BUS busid)
{
	struct ms5837_bus_option &bus = find_bus(busid);
	int fd;

	fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct ms5837_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			warnx("%s", bus.devpath);
			bus.dev->print_info();
		}
	}

	exit(0);
}

/**
 * Calculate actual MSL pressure given current altitude
 */
void
calibrate(unsigned altitude, enum MS5837_BUS busid)
{	
	struct ms5837_bus_option &bus = find_bus(busid);
	struct sensor_depth_s report;
	float	pressure;
	float	p1;

	int fd;



	fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		err(1, "open failed (try 'ms5837 start' if the driver is not running)");
	}

	/* start the sensor polling at max */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX)) {
		errx(1, "failed to set poll rate");
	}

	/* average a few measurements */
	pressure = 0.0f;

	for (unsigned i = 0; i < 20; i++) {
		struct pollfd fds;
		int ret;
		ssize_t sz;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 1000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "sensor read failed");
		}

		pressure += report.abs_pressure;
	}

	pressure /= 20;		/* average */
	pressure /= 10;		/* scale from millibar to kPa */

	const float g  = 9.80665f;	/* gravity constant in m/s/s */
	float rho      = (float)(ioctl(fd, BAROIOCGWATERDENSITY, 0)/10000.0f);

	warnx("using water density %10.4f", (double)rho);
	warnx("averaged pressure %10.4fkPa at %um", (double)pressure, altitude);

	p1 = pressure - (rho * g * altitude);

	warnx("calculated MSL pressure %10.4fkPa", (double)p1);

	/* save as integer Pa */
	p1 *= 1000.0f;

	if (ioctl(fd, BAROIOCSMSLPRESSURE, (unsigned long)p1) != OK) {
		err(1, "BAROIOCSMSLPRESSURE");
	}

	close(fd);
	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'test2', 'reset', 'calibrate'");
	warnx("options:");
	warnx("    -X    (external I2C bus) (default)");
	warnx("    -I    (intternal I2C bus)");

}

} // namespace

int
ms5837_main(int argc, char *argv[])
{
	enum MS5837_BUS busid = MS5837_BUS_I2C_EXTERNAL;
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "T:XI", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = MS5837_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = MS5837_BUS_I2C_INTERNAL;
			break;

		//no break
		default:
			ms5837::usage();
			exit(0);
		}
	}


	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		ms5837::start(busid);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		ms5837::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		ms5837::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		ms5837::info();
	}

	/*
	 * Perform MSL pressure calibration given an altitude in metres
	 */
	if (!strcmp(verb, "calibrate")) {
		if (argc < 2) {
			errx(1, "missing altitude");
		}

		long altitude = strtol(argv[optind + 1], nullptr, 10);

		ms5837::calibrate(altitude, busid);
	}

	ms5837::usage();
	exit(0);
}
