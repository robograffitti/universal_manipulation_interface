import os
import time
import enum
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
import scipy.interpolate as si
import scipy.spatial.transform as st
import numpy as np

from umi.shared_memory.shared_memory_queue import (
    SharedMemoryQueue, Empty)
from umi.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from umi.common.pose_trajectory_interpolator import PoseTrajectoryInterpolator
from diffusion_policy.common.precise_sleep import precise_wait
import torch
from umi.common.pose_util import pose_to_mat, mat_to_pose
# import zerorpc

# write some moveit related import
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class Command(enum.Enum):
    STOP = 0
    SERVOL = 1
    SCHEDULE_WAYPOINT = 2

class XArmInterface:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("xarm_move_group_interface", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "xarm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_pub = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)
        group_names = robot.get_group_names()
        print("============ End effector link: %s" % eef_link)
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_pub = display_trajectory_pub
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def get_ee_pose(self):
        pose = self.move_group.get_current_pose().pose
        pose.position.
        return pose # convert to ndarray
    
    def get_joint_positions(self):
        joint_positions = self.move_group.get_current_joint_values()
        return joint_positions # array with 7 elements
        
    def update_desired_ee_pose(self, pose: np.ndarray):
        # self.server.update_desired_ee_pose(pose.tolist())
        pose_goal = pose # convert pose to 
        self.move_group.set_pose_target(pose_goal)

class XArmInterpolationController(mp.Process):

    def __init__(self,
        shm_manager: SharedMemoryManager, 
        frequency=1000,
        launch_timeout=3,
        joints_init=None,
        joints_init_duration=None,
        soft_real_time=False,
        verbose=False,
        get_max_k=None,
        receive_latency=0.0
        ):
        """
        robot_ip: the ip of the middle-layer controller (NUC)
        frequency: 1000 for franka
        Kx_scale: the scale of position gains
        Kxd: the scale of velocity gains
        soft_real_time: enables round-robin scheduling and real-time priority
            requires running scripts/rtprio_setup.sh before hand.
        """

        super().__init__(name="XArmPositionalController")
        self.frequency = frequency
        self.launch_timeout = launch_timeout
        self.soft_real_time = soft_real_time
        self.receive_latency = receive_latency
        self.verbose = verbose

        if get_max_k is None:
            get_max_k = int(frequency * 5)

        example = {
            'cmd': Command.SERVOL.value,
            'target_pose': np.zeros((6,), dtype=np.float64),
            'duration': 0.0,
            'target_time': 0.0
        }
        input_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            buffer_size=256
        )

        # build ring buffer
        receive_keys = [
            ('ActualTCPPose', 'get_ee_pose'),
            ('ActualQ', 'get_joint_positions'),
            ('ActualQd','get_joint_velocities'),
        ]
        example = dict()
        for key, func_name in receive_keys:
            if 'joint' in func_name:
                example[key] = np.zeros(7)
            elif 'ee_pose' in func_name:
                example[key] = np.zeros(6)

        example['robot_receive_timestamp'] = time.time()
        example['robot_timestamp'] = time.time()
        ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )

        self.ready_event = mp.Event()
        self.input_queue = input_queue
        self.ring_buffer = ring_buffer
        self.receive_keys = receive_keys

    def start(self, wait=True):
        super().start()
        if wait:
            self.start_wait()
        if self.verbose:
            print(f"[XArmPositionalController] Controller process spawned at {self.pid}")
    
    def stop(self, wait=True):
        message = {
            'cmd': Command.STOP.value
        }
        self.input_queue.put(message)
        if wait:
            self.stop_wait()
    
    def start_wait(self):
        self.ready_event.wait(self.launch_timeout)
        assert self.is_alive()
    
    def stop_wait(self):
        self.join()
    
    @property
    def is_ready(self):
        return self.ready_event.is_set()
    
    # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ========= command methods ============
    def servoL(self, pose, duration=0.1):
        """
        duration: desired time to reach pose
        """
        assert self.is_alive()
        assert(duration >= (1/self.frequency))
        pose = np.array(pose)
        assert pose.shape == (6,)

        message = {
            'cmd': Command.SERVOL.value,
            'target_pose': pose,
            'duration': duration
        }
        self.input_queue.put(message)
    
    def schedule_waypoint(self, pose, target_time):
        pose = np.array(pose)
        assert pose.shape == (6,)

        message = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pose': pose,
            'target_time': target_time
        }
        self.input_queue.put(message)

    # ========= receive APIs =============
    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k,out=out)
    
    def get_all_state(self):
        return self.ring_buffer.get_all()
    
    # ========= main loop in process ============
    def run(self):
        # enable soft real-time
        if self.soft_real_time:
            os.sched_setscheduler(
                0, os.SCHED_RR, os.sched_param(20))
            
        # start polymetis interface
        robot = XArmInterface()

        # start rospy

        try:
            # if self.verbose:
            #     print(f"[RTDEPositionalController] Connect to robot: {robot_ip}")

            # set parameters

            # init pose

            # main loop
            # rospy.init_node('xarm_interpolation_controller', anonymous=True)
            # rate = rospy.Rate(self.frequency) # 10hz

            # pub =
            # sub =


            keep_running = True
            while keep_running: # use instead of rospy.is_shutdown() ?
            # while not rospy.is_shutdown():
                # send command to robot

                # send command to robot
                # robot.update_desired_ee_pose() # moveit desired pose
                # move_group.set_pose_target(pose_goal)

                # update robot state

                # fetch command from queue
                # try:
                # except Empty:
                # rate.sleep()
                pass

        except rospy.ROSInterruptException as err:
            rospy.signal_shutdown(err)

        finally:
            rospy.signal_shutdown("Successfully terminated!")
    
    def callback(self, data):
        pass
