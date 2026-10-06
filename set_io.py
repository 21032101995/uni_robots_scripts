#!/usr/bin/env python

import rospy
from ur_msgs.srv import SetIO, SetIORequest

def toggle_digital_output():
    rospy.init_node('ur_do1_toggler', anonymous=True)

    # Service name for UR hardware interface
    service_name = '/ur_hardware_interface/set_io'
    
    rospy.loginfo(f"Waiting for service {service_name}...")
    rospy.wait_for_service(service_name)
    
    set_io = rospy.ServiceProxy(service_name, SetIO)

    # Pin configuration
    # SetIO fun: 1 = PIN_LEVEL (digital output)
    # pin: 1 for DO1 (use 0 for DO0)
    pin_index = 1
    fun_digital_out = 1

    rospy.loginfo("Starting digital output toggle loop. Press Ctrl+C to stop.")

    rate = rospy.Rate(1)  # 1 Hz delay handling if needed, or explicit rospy.sleep

    while not rospy.is_shutdown():
        try:
            # 1. Activate DO1 (state = 1.0)
            rospy.loginfo("Activating DO1...")
            set_io(fun=fun_digital_out, pin=pin_index, state=1.0)
            rospy.sleep(1.0)

            # 2. Deactivate DO1 (state = 0.0)
            rospy.loginfo("Deactivating DO1...")
            set_io(fun=fun_digital_out, pin=pin_index, state=0.0)
            rospy.sleep(1.0)

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            break

if __name__ == '__main__':
    try:
        toggle_digital_output()
    except rospy.ROSInterruptException:
        pass