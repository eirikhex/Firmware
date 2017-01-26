#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int bent_app_2_main(int argc, char *argv[]);

int bent_app_2_main(int argc, char *argv[])
{
    PX4_INFO("Hello Bent 2!");
    return OK;
}
