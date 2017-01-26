#ifndef ROVCONTROL_ROV_CONTROL_H
#define ROVCONTROL_ROV_CONTROL_H

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <uORB/Subscription.hpp>

class ROVControl
{
public:
	ROVControl();
	~ROVControl();
	int start();
	void exit() {_task_should_exit = true;}


private:
	bool	_task_should_exit;	/**< if true, task_main() should exit */
	bool	_task_running;		/**< if true, task is running in its mainloop */
	int	_control_task;		/**< task handle */
	int	_params_sub; 		/**< parameter updates subscription */
	int	_manual_control_sp_sub;	/**< manual control setpoint subscription */
	int	_armed_sub;		/**< arming status subscription */
    //double[] _thrust_allocation; //Thrust allocation

	orb_advert_t	_actuators_0_pub;	/**< attitude actuator controls publication */
	orb_id_t 	_actuators_id;		/**< pointer to correct actuator controls0 uORB metadata structure */

	struct manual_control_setpoint_s   _manual_control_sp;	/**< manual control setpoint */
	struct actuator_controls_s	   _actuators;	/**< actuator controls */
	struct actuator_armed_s		   _armed;		/**< actuator arming status */
	
	perf_counter_t	_loop_perf;			/**< loop performance counter */


	void		task_main();     				//Main attitude control task.    
	void		vehicle_manual_poll(); 				// Check for changes in manual inputs
	void 		arming_status_poll();  				// Check for arming status updates.
	int  		parameters_update();   				//Update our local parameter cache.
	static void	task_main_trampoline(int argc, char *argv[]);   // Shim for calling task_main from task_create.
    //double[]    thrust_allocation(double[]); //Calculates the force to be produced by each thruster (min 0, max 1), based on a given (desired) force vector: the desired force vector tells us what forces we want the ROV to produce in all directions and orientations.

};
#endif

