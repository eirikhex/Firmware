/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file rov_control.cpp
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/uORB.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/debug_key_value.h>

#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>

extern "C" __EXPORT int rov_control_main(int argc, char *argv[]);

class ROVControl
{
public:
		/**
	 * Constructor
	 */
	ROVControl();

	/**
	 * Destructor, also kills the main task
	 */
	~ROVControl();

	/**
	 * Start the rov control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	void exit() { _task_should_exit = true; }

private:
	bool	_task_should_exit;		/**< if true, task_main() should exit */
	bool	_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle */

	int		_params_sub;			/**< parameter updates subscription */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
	int		_armed_sub;				/**< arming status subscription */

	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */

	orb_id_t _actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */

	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct actuator_controls_s			_actuators;			/**< actuator controls */
	struct actuator_armed_s				_armed;				/**< actuator arming status */
	

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for arming status updates.
	 */
	void		arming_status_poll();

		/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();
};

namespace rov_control
{

ROVControl	*instance = nullptr;
}

ROVControl::ROVControl():
	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),

	/* subscriptions */
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),

	/* publications */
	_actuators_0_pub(nullptr),
	_actuators_id(0),

	_manual_control_sp{},
	_actuators{},
	_armed{},

		/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "rov_c_dt"))
{

}

ROVControl::~ROVControl()
{	

	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	perf_free(_loop_perf);
	rov_control::instance = nullptr;
	
}

void
ROVControl::task_main()
{
	/* initialization */
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));


	_actuators_id = ORB_ID(actuator_controls_0);


	vehicle_manual_poll();
	arming_status_poll();
	parameters_update();


	/* wakeup source */
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _manual_control_sp_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _params_sub;
	fds[1].events = POLLIN;
	

	struct debug_key_value_s dbg = {hrt_absolute_time(), 0, 0.0f, "debug" ,0 }; 
	orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);

	_task_running = true;

	/*main loop*/
	while (!_task_should_exit) {


		/* wait for up to 100ms for data */

		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);


		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			usleep(200);
			continue;
		}


		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}


		perf_begin(_loop_perf);

		/* only update parameters if they changed */
		if (fds[1].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[0].revents & POLLIN) {
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f) {
				deltaT = 0.01f;
			}

			vehicle_manual_poll();

			_actuators.control[0] = _manual_control_sp.x;
			_actuators.control[1] = _manual_control_sp.y;
			_actuators.control[2] = _manual_control_sp.z;
			_actuators.control[3] = 0.0;
			_actuators.control[4] = 0.0;
			_actuators.control[5] = _manual_control_sp.r;
			_actuators.timestamp = hrt_absolute_time();

			usleep(100);

			/* publish actuator data*/
			if (_actuators_0_pub != nullptr) {

			} 
			else if (_actuators_id) {
				_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
				dbg.value = (float)999.09;
				orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
				orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
			} 
			else if (_actuators_id) {
				_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
			}
		}

		


		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;

}

void
ROVControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
ROVControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

int
ROVControl::parameters_update()
{
	return PX4_OK;
}

void
ROVControl::task_main_trampoline(int argc, char *argv[])
{
	rov_control::instance->task_main();
}

int
ROVControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("rov_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&ROVControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int rov_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_WARN("usage: rov_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (rov_control::instance != nullptr) {
			PX4_WARN("already running");
			return 1;
		}

		rov_control::instance = new ROVControl();

		if (rov_control::instance == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		if (OK != rov_control::instance->start()) {
			delete rov_control::instance;
			rov_control::instance = nullptr;
			PX4_WARN("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (rov_control::instance == nullptr) {
			PX4_WARN("not running");
			return 1;
		}

		delete rov_control::instance;
		rov_control::instance = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "print")) {
		if (rov_control::instance != nullptr) {

			return 0;
		}

		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		if (rov_control::instance) {
			PX4_WARN("running");
			return 0;

		} else {
			PX4_WARN("not running");
			return 1;
		}
	}

	PX4_WARN("unrecognized command");
	return 1;
}



