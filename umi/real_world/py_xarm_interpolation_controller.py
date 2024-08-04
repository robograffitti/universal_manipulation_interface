import os
import sys
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

from umi.common.pose_util import pose_to_mat, mat_to_pose

from xarm.wrapper import XArmAPI

class XArmInterface:
    def __init__(self, ip='192.168.10.211'):
        arm = XArmAPI(ip, is_radian=True)
        arm.motion_enable(enable=True)
        arm.set_mode(0)
        arm.set_state(state=0)
        arm.reset(wait=True)
        self.robot = arm

    def get_ee_pose(self):
        data = self.robot.get_position()
        code = data[0]
        pose = data[1]
        pos = pose[:3]
        rpy = pos[3:]
        return 

class Command(enum.Enum):
    STOP = 0
    SERVOL = 1
    SCHEDULE_WAYPOINT = 2

class XArmInterpolationController(mp.Process):

    def __init__(self,
        shm_manager: SharedMemoryManager, 
        robot_ip,
        frequency=100, # or 250hz
        launch_timeout=3,
        joints_init=None,
        joints_init_speed=10,
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

        if joints_init is not None:
            joints_init = np.array(joints_init)
            assert joints_init.shape == (7,)

        super().__init__(name="PyXArmPositionalController")
        self.robot_ip = robot_ip
        self.frequency = frequency
        self.launch_timeout = launch_timeout
        self.joints_init = joints_init
        self.joints_init_speed = joints_init_speed
        self.soft_real_time = soft_real_time
        self.receive_latency = receive_latency
        self.verbose = verbose

        if get_max_k is None:
            get_max_k = int(frequency * 5)

        # build input queue
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
