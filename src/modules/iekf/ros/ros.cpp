#include "ros.hpp"
#include "px4_posix.h"
#include "drivers/drv_hrt.h"

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/att_pos_mocap.h>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/vehicle_land_detected.h>

namespace ros
{

HashMap<const char *, const struct orb_metadata *, 128> _topicDict;
HashMap<const char *, uint32_t, 128> _paramDict;

Time::Time() : _nsec(0)
{
}

Time::Time(uint64_t nsec) : _nsec(nsec)
{
}

Time Time::fromNSec(uint64_t nsec)
{
	return Time(nsec);
}

uint64_t Time::toNSec()
{
	return _nsec;
}

Time Time::now()
{
	return fromNSec(1e3 * hrt_absolute_time());
}

// The node for this process
Node _node_singleton;
Node *_node = NULL;

void init(int argc, char **argv, const char *node_name)
{
	_node = &_node_singleton;
}

void spin()
{
	if (_node != NULL) {
		//ROS_INFO("node spinning");
		_node->spin();

	} else {
		ROS_INFO("node not initialized");
	}
}

Node::Node() :
	_callbackListHead(NULL)
{
	_topicDict.put("sensor_combined", ORB_ID(sensor_combined));
	_topicDict.put("vehicle_gps_position", ORB_ID(vehicle_gps_position));
	_topicDict.put("airspeed", ORB_ID(airspeed));
	_topicDict.put("optical_flow", ORB_ID(optical_flow));
	_topicDict.put("distance_sensor", ORB_ID(distance_sensor));
	_topicDict.put("vision_position_estimate", ORB_ID(vision_position_estimate));
	_topicDict.put("att_pos_mocap", ORB_ID(att_pos_mocap));
	_topicDict.put("vehicle_attitude", ORB_ID(vehicle_attitude));
	_topicDict.put("vehicle_local_position", ORB_ID(vehicle_local_position));
	_topicDict.put("vehicle_global_position", ORB_ID(vehicle_global_position));
	_topicDict.put("control_state", ORB_ID(control_state));
	_topicDict.put("estimator_status", ORB_ID(estimator_status));
	_topicDict.put("vehicle_land_detected", ORB_ID(vehicle_land_detected));
}

void Node::spin()
{
	CallbackInterface *tail = _callbackListHead;

	while (tail != NULL) {
		tail->callback();
		tail = tail->next;
	}
}

void Node::addSubscriber(Subscriber *sub)
{
	if (_callbackListHead == NULL) {
		_callbackListHead = sub->_callbackPtr;
		return;
	}

	CallbackInterface *tail = _callbackListHead;

	while (tail->next != NULL) {
		tail = tail->next;
	}

	tail->next = sub->_callbackPtr;
}

bool Node::getTopicMeta(const char *topic, const struct orb_metadata **meta)
{
	return _topicDict.get(topic, *meta);
}


Rate::Rate(float frequency):
	_frequency(frequency),
	_wake_timestamp(hrt_absolute_time())
{
}

void Rate::sleep()
{
	int32_t dt = _wake_timestamp - hrt_absolute_time();

	if (dt > 0) {
		usleep(dt);
	}

	_wake_timestamp = hrt_absolute_time() + 1.0e6f / _frequency;
}

Subscriber::Subscriber() :
	_callbackPtr(NULL)
{
}

Subscriber::Subscriber(const Subscriber &sub) :
	_callbackPtr(sub._callbackPtr)
{
};

Subscriber::Subscriber(CallbackInterface *cb) :
	_callbackPtr(cb)
{
	if (_node != NULL) {
		_node->addSubscriber(this);

	} else {
		ROS_INFO("node not initialized");
	}
}


Subscriber &Subscriber::operator=(const Subscriber &sub)
{
	_callbackPtr = sub._callbackPtr;
	return *this;
}

Subscriber::~Subscriber()
{
}

void Subscriber::callback()
{
	if (_callbackPtr != NULL) {
		_callbackPtr->callback();

	} else {
		ROS_INFO("callback ptr is NULL!");
	}
}

Publisher::Publisher(uORB::PublicationTiny *pub) :
	_pub(pub)
{
}

Publisher::Publisher(const Publisher &pub) :
	_pub(pub._pub)
{
};


Publisher &Publisher::operator=(const Publisher &pub)
{
	_pub = pub._pub;
	return *this;
}

Publisher::~Publisher()
{
}

bool NodeHandle::ok()
{
	return true;
}

}
