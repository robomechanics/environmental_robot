#!/usr/bin/env python3

import rospy
import hebi
from time import sleep
import numpy as np
from std_srvs.srv import Trigger, TriggerResponse
 
class Tool_Arm_Controller:
    def __init__(self):
        lookup = hebi.Lookup()
        sleep(2)

        self.hebiFamily = ['R8-16']
        self.hebiName = ['JR_rototiller']
        self.hebiGroup = lookup.get_group_from_names(self.hebiFamily,self.hebiName)

        self.thetaUp = -0.90
        self.thetaDown = -2.40
        self.thetaOffset = 0.1
        self.dt = 0.1
        self.duration = 1.5
        self.times = np.linspace(0.0,self.duration,2)
        self.numActuators = 1

        self.arm_return_service = rospy.Service(
            "tool_up", Trigger, self.tool_up
        )
        
        self.arm_touchdown_service = rospy.Service(
            "tool_down", Trigger, self.tool_down
        )

        self.tool_behavior_service = rospy.Service(
            "tool_behavior", Trigger, self.tool_behavior
        )

    def tool_up(self, req):
        rows = 1 # num of actuators
        cols = 2 # num of target waypoints
        pos = np.zeros((rows,cols))
        
        pos[:,0] = [self.thetaDown]
        pos[:,1] = [self.thetaUp]

        vel = np.full(pos.shape, np.nan)
        acc = np.full(pos.shape, np.nan)

        # Set first and last waypoint values to 0.0
        vel[:, 0] = acc[:, 0] = 0.0
        vel[:, -1] = acc[:, -1] = 0.0

        # Create the trajectory
        self.trajectory = hebi.trajectory.create_trajectory(
            self.times, np.copy(pos), vel, acc
        )

        cmd = hebi.GroupCommand(self.numActuators)

        t = 0.0
        duration = self.trajectory.duration

        pos_cmd = np.array(self.numActuators, dtype=np.float64)
        vel_cmd = np.array(self.numActuators, dtype=np.float64)


        while t < duration:
            self.pos_cmd, self.vel_cmd, self.acc_cmd = self.trajectory.get_state(t)
            
            cmd.position = self.pos_cmd
            cmd.velocity = self.vel_cmd

            self.hebiGroup.command_lifetime = 0
            
            self.hebiGroup.send_command(cmd)

            t = t + self.dt
            sleep(self.dt)

        return TriggerResponse(success=True, message="Tool arm went up!")

    def tool_down(self, req):
        rows = 1 # num of actuators
        cols = 2 # num of target waypoints
        pos = np.zeros((rows,cols))
        
        pos[:,0] = [self.thetaUp]
        pos[:,1] = [self.thetaDown]

        vel = np.full(pos.shape, np.nan)
        acc = np.full(pos.shape, np.nan)

        # Set first and last waypoint values to 0.0
        vel[:, 0] = acc[:, 0] = 0.0
        vel[:, -1] = acc[:, -1] = 0.0

        # Create the trajectory
        self.trajectory = hebi.trajectory.create_trajectory(
            self.times, np.copy(pos), vel, acc
        )

        cmd = hebi.GroupCommand(self.numActuators)

        t = 0.0
        duration = self.trajectory.duration

        pos_cmd = np.array(self.numActuators, dtype=np.float64)
        vel_cmd = np.array(self.numActuators, dtype=np.float64)

        while t < duration:
            self.pos_cmd, self.vel_cmd, self.acc_cmd = self.trajectory.get_state(t)
            
            cmd.position = self.pos_cmd
            cmd.velocity = self.vel_cmd

            self.hebiGroup.command_lifetime = 0
            
            self.hebiGroup.send_command(cmd)

            t = t + self.dt
            sleep(self.dt)


        return TriggerResponse(success=True, message="Tool arm went down!")
    
    def tool_behavior(self, req):
        rows = 1 # num of actuators
        cols = 3 # num of target waypoints
        pos = np.zeros((rows,cols))
        
        pos[:,0] = [self.thetaUp]
        pos[:,1] = [self.thetaDown]
        pos[:,2] = [self.thetaUp]

        vel = np.full(pos.shape, np.nan)
        acc = np.full(pos.shape, np.nan)

        # Set first and last waypoint values to 0.0
        vel[:, 0] = acc[:, 0] = 0.0
        vel[:, -1] = acc[:, -1] = 0.0

        # Create the trajectory
        self.trajectory = hebi.trajectory.create_trajectory(
            self.times, np.copy(pos), vel, acc
        )

        cmd = hebi.GroupCommand(self.numActuators)

        t = 0.0
        duration = self.trajectory.duration

        pos_cmd = np.array(self.numActuators, dtype=np.float64)
        vel_cmd = np.array(self.numActuators, dtype=np.float64)

        while t < duration:
            self.pos_cmd, self.vel_cmd, self.acc_cmd = self.trajectory.get_state(t)
            
            cmd.position = self.pos_cmd
            cmd.velocity = self.vel_cmd

            self.hebiGroup.command_lifetime = 0
            
            self.hebiGroup.send_command(cmd)

            t = t + self.dt
            sleep(self.dt)


        return TriggerResponse(success=True, message="Tool service successfully!")


if __name__ == "__main__":

    rospy.init_node('tool_arm_control')
    controller = Tool_Arm_Controller()

    rospy.spin()