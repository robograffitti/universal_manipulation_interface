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
# import zerorpc # for franka only

# write some moveit related import
import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class Command(enum.Enum):
    STOP = 0
    SERVOL = 1
    SCHEDULE_WAYPOINT = 2

# [IN PROGRESS] tip pose calculation
# tx_flangerot90_tip = np.identity(4)
# tx_flangerot90_tip[:3, 3] = np.array([-0.0336, 0, 0.247])
# tx_flangerot45_flangerot90 = np.identity(4)
# tx_flangerot45_flangerot90[:3,:3] = st.Rotation.from_euler('x', [np.pi/2]).as_matrix()
# tx_flange_flangerot45 = np.identity(4)
# tx_flange_flangerot45[:3,:3] = st.Rotation.from_euler('z', [np.pi/4]).as_matrix()
# tx_flange_tip = tx_flange_flangerot45 @ tx_flangerot45_flangerot90 @tx_flangerot90_tip
# tx_tip_flange = np.linalg.inv(tx_flange_tip)

class XArmInterface:
    
    def __init__(self):

        # Initialize move_group_interface
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("xarm_move_group_interface", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "xarm7"
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
        print("============ Available Planning Groups:", group_names)
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
        pose = self.move_group.get_current_pose()
        (r, p, y) = tf.transformations.euler_from_quaternion(pose[3:])
        pose = [pose[0], pose[1], pose[2], r, p, y]
        return np.array(pose)        
    
    def get_joint_positions(self):
        joint_positions = self.move_group.get_current_joint_values()
        return np.array(joint_positions) # array with 7 elements
    
    def get_joint_velocities(self):
        joint_state = self.move_group.get_current_state()
        joint_velocities = joint_state.velocity
        return np.array(joint_velocities)
    
    def move_to_joint_positions(self, positions: np.ndarray, time_to_go: float):
        joint_goal = positions.tolist()
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        
    # https://robo-marc.github.io/moveit_documents/moveit_commander.html
    def update_desired_ee_pose(self, pose: np.ndarray):
        # self.server.update_desired_ee_pose(pose.tolist())
        # pose_goal = pose # convert pose to 
        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.position
        # pose_goal.orientation
        # self.move_group.set_pose_target(pose_goal)
        # success = self.move_group.go(wait=True)
        self.move_group.set_pose_target(pose.tolist()) # Pose, PoseStamped, or list
        self.move_group.go(wait=True)
        self.move_group.clear_pose_targets()
        self.move_group.stop()
    
    def stop(self): # or close?
        self.move_group.stop()

class XArmInterpolationController(mp.Process):

    def __init__(self,
        shm_manager: SharedMemoryManager, 
        frequency=125,
        launch_timeout=3,
        joints_init=None,
        joints_init_duration=None,
        soft_real_time=False,
        verbose=False,
        get_max_k=None,
        receive_latency=0.0
        ):
        """
        frequency: 1000 for franka
        soft_real_time: enables round-robin scheduling and real-time priority
            requires running scripts/rtprio_setup.sh before hand.
        """

        if joints_init is not None:
            joints_init = np.array(joints_init)
            assert joints_init.shape == (7,)

        super().__init__(name="XArmPositionalController")
        self.frequency = frequency
        self.launch_timeout = launch_timeout
        self.joints_init = joints_init
        self.joints_init_duration = joints_init_duration
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

    # ========= launch method ===========
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

        try:
            if self.verbose:
                print(f"[XArmPositionalController] Connect to robot: {self.robot_ip}")
            
            # init pose
            if self.joints_init is not None:
                robot.move_to_joint_positions(
                    positions=np.asarray(self.joints_init),
                    time_to_go=self.joints_init_duration
                )

            # main loop
            dt = 1. / self.frequency
            curr_pose = robot.get_ee_pose()

            # use monotonic time to make sure the control loop never go backward
            curr_t = time.monotonic()
            last_waypoint_time = curr_t
            pose_interp = PoseTrajectoryInterpolator(
                times=[curr_t],
                poses=[curr_pose]
            )

            t_start = time.monotonic()
            iter_idx = 0
            keep_running = True
            while keep_running:
                # send command to robot
                t_now = time.monotonic()
                # diff = t_now - pose_interp.times[-1]
                # if diff > 0:
                #     print('extrapolate', diff)

                # [IN PROGRESS] EE pose conversion, calculation
                pose_command = pose_interp(t_now) # np.ndarray
                # flange_pose = mat_to_pose(pose_to_mat(tip_pose) @ tx_tip_flange)
                # [IN PROGRESS] send command to robot
                # robot.update_desired_ee_pose(flange_pose)
                robot.update_desired_ee_pose(pose_command)

                # update robot state [OK]
                state = dict()
                for key, func_name in self.receive_keys:
                    state[key] = getattr(robot, func_name)()
                
                t_recv = time.time()
                state['robot_receive_timestamp'] = t_recv
                state['robot_timestamp'] = t_recv - self.receive_latency
                self.ring_buffer.put(state)

                # fetch command from queue
                try:
                    # commands = self.input_queue.get_all()
                    # n_cmd = len(commands['cmd'])
                    # process at most 1 command per cycle to maintain frequency
                    commands = self.input_queue.get_k(1)
                    n_cmd = len(commands['cmd'])
                except Empty:
                    n_cmd = 0

                # execute commands
                for i in range(n_cmd):
                    command = dict()
                    for key, value in commands.items():
                        command[key] = value[i]
                    cmd = command['cmd']

                    if cmd == Command.STOP.value:
                        keep_running = False
                        # stop immediately, ignore later commands
                        break
                    elif cmd == Command.SERVOL.value:
                        # since curr_pose always lag behind curr_target_pose
                        # if we start the next interpolation with curr_pose
                        # the command robot receive will have discontinouity 
                        # and cause jittery robot behavior.
                        target_pose = command['target_pose']
                        duration = float(command['duration'])
                        curr_time = t_now + dt
                        t_insert = curr_time + duration
                        pose_interp = pose_interp.drive_to_waypoint(
                            pose=target_pose,
                            time=t_insert,
                            curr_time=curr_time,
                        )
                        last_waypoint_time = t_insert
                        if self.verbose:
                            print("[XArmPositionalController] New pose target:{} duration:{}s".format(
                                target_pose, duration))
                    elif cmd == Command.SCHEDULE_WAYPOINT.value:
                        target_pose = command['target_pose']
                        target_time = float(command['target_time'])
                        # translate global time to monotonic time
                        target_time = time.monotonic() - time.time() + target_time
                        curr_time = t_now + dt
                        pose_interp = pose_interp.schedule_waypoint(
                            pose=target_pose,
                            time=target_time,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time
                        )
                        last_waypoint_time = target_time
                    else:
                        keep_running = False
                        break

                # regulate frequency
                t_wait_util = t_start + (iter_idx + 1) * dt
                precise_wait(t_wait_util, time_func=time.monotonic)

                # first loop successful, ready to receive command
                if iter_idx == 0:
                    self.ready_event.set()
                iter_idx += 1

                if self.verbose:
                    print(f"[XArmPositionalController] Actual frequency {1/(time.monotonic() - t_now)}")

        except rospy.ROSInterruptException: # pass
            return
        
        except KeyboardInterrupt:
            rospy.signal_shutdown("Keyboard Interruption!")
            return

        finally:
            rospy.signal_shutdown("Successfully terminated!")
            del robot
            self.ready_event.set()

            if self.verbose:
                print(f"[XArmPositionalController] Disconnected from robot.")