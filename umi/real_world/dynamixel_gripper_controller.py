import os
import time
import enum
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
from umi.shared_memory.shared_memory_queue import (
    SharedMemoryQueue, Empty)
from umi.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from umi.common.precise_sleep import precise_wait
from umi.real_world.wsg_binary_driver import WSGBinaryDriver
from umi.common.pose_trajectory_interpolator import PoseTrajectoryInterpolator

# for gripper
import sys
import rospy
# import time
import math
import actionlib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

class Command(enum.Enum):
    SHUTDOWN = 0
    SCHEDULE_WAYPOINT = 1
    RESTART_PUT = 2

# https://github.com/ryuichiueda/my_crane_x7_samples/blob/master/scripts/gripper_action_example.py
class GripperClient(object):
    def __init__(self):
        # アクションクライアント（クライアント）を作っている
        # クライアントはアクションサーバにゴール（目標の姿勢）を投げて実行してもらったり、
        # 途中で動作を中断してもらったりする。
        self._client = actionlib.SimpleActionClient("/crane_x7/gripper_controller/gripper_cmd",GripperCommandAction)
        self.clear() # 不要かもしれないけどゴールを初期化

        # アクションサーバを待つ（10秒でタイムアウト）
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)

    # アクションサーバにゴールを送るメソッド
    def command(self, position, effort):
        self._goal.command.position = position # グリッパの姿勢（開く角度）
        self._goal.command.max_effort = effort # トルク
        self._client.send_goal(self._goal,feedback_cb=self.feedback) # ゴールをサーバに送信

    # サーバが仕事をしている途中で呼ばれるメソッド（このサンプルでは呼ばれてない）
    def feedback(self,msg):
        print("feedback callback")
        print(msg)

    # 途中で中断するためのメソッド（これもこのサンプルでは使われていない）
    def stop(self):
        self._client.cancel_goal()

    # サーバからの結果待ちのためのメソッド
    def wait(self, timeout=0.1):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result() # 結果をもらう

    # ゴール情報のクリアのためのメソッド
    def clear(self):
        self._goal = GripperCommandGoal()

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        self.clear()

class DynamixelGripperController(mp.Process):
    def __init__(self,
            shm_manager: SharedMemoryManager,
            # hostname,
            # port=1000,
            frequency=30,
            home_to_open=True,
            move_max_speed=200.0,
            get_max_k=None,
            command_queue_size=1024,
            launch_timeout=3,
            receive_latency=0.0,
            use_meters=False, # not sure
            verbose=False
            ):
        super().__init__(name="DynamixelGripperController")
        # self.hostname = hostname
        # self.port = port
        self.frequency = frequency
        self.home_to_open = home_to_open
        self.move_max_speed = move_max_speed
        self.launch_timeout = launch_timeout
        self.receive_latency = receive_latency
        self.scale = 1000.0 if use_meters else 1.0
        self.verbose = verbose

        if get_max_k is None:
            get_max_k = int(frequency * 10)
        
        # build input queue
        example = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pos': 0.0,
            'target_time': 0.0
        }
        input_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            buffer_size=command_queue_size
        )
        
        # build ring buffer
        example = {
            'gripper_state': 0,
            'gripper_position': 0.0,
            'gripper_velocity': 0.0,
            'gripper_force': 0.0,
            'gripper_measure_timestamp': time.time(),
            'gripper_receive_timestamp': time.time(),
            'gripper_timestamp': time.time()
        }
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

    # ========= launch method ===========
    def start(self, wait=True):
        super().start()
        if wait:
            self.start_wait()
        if self.verbose:
            print(f"[DynamixelGripperController] Controller process spawned at {self.pid}")

    def stop(self, wait=True):
        message = {
            'cmd': Command.SHUTDOWN.value
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
    def schedule_waypoint(self, pos: float, target_time: float):
        message = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pos': pos,
            'target_time': target_time
        }
        self.input_queue.put(message)


    def restart_put(self, start_time):
        self.input_queue.put({
            'cmd': Command.RESTART_PUT.value,
            'target_time': start_time
        })
    
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
        # start connection
        try:
            # with WSGBinaryDriver(
            #     hostname=self.hostname, 
            #     port=self.port) as wsg:
                
            #     # home gripper to initialize
            #     wsg.ack_fault()
            #     wsg.homing(positive_direction=self.home_to_open, wait=True)

            #     # get initial
            #     curr_info = wsg.script_query()
            #     curr_pos = curr_info['position']
            
            rospy.init_node("gripper_action_client")
            
            with GripperClient() as gc:
                curr_msg = rospy.wait_for_message('/joint_states', JointState, timeout=None)
                curr_pos = curr_msg.position[7]
                curr_t = time.monotonic()
                last_waypoint_time = curr_t
                pose_interp = PoseTrajectoryInterpolator(
                    times=[curr_t],
                    poses=[[curr_pos,0,0,0,0,0]]
                )
                
                keep_running = True
                t_start = time.monotonic()
                iter_idx = 0
                while keep_running:
                    # command gripper
                    t_now = time.monotonic()
                    dt = 1 / self.frequency
                    t_target = t_now
                    target_pos = pose_interp(t_target)[0]
                    max_pos = 0.15 # tmp
                    max_angle = 0.125 # tmp
                    target_angle = target_pos / max_pos * max_angle
                    target_vel = (target_pos - pose_interp(t_target - dt)[0]) / dt
                    max_effort = 1.0 # tmp
                    gc.command(target_angle, max_effort)
                    # print('controller', target_pos, target_vel)
                    # info = wsg.script_position_pd(
                    #     position=target_pos, velocity=target_vel)
                    # time.sleep(1e-3)

                    
                    # curr_state = rospy.wait_for_message('/joint_states', JointState, timeout=None)
                    info = rospy.wait_for_message('/joint_states', JointState, timeout=None)

                    # get state from robot
                    # need modification
                    state = {
                        'gripper_state': info['state'],
                        'gripper_position': info['position'] / self.scale,
                        'gripper_velocity': info['velocity'] / self.scale,
                        'gripper_force': info['force_motor'],
                        'gripper_measure_timestamp': info['measure_timestamp'],
                        'gripper_receive_timestamp': time.time(),
                        'gripper_timestamp': time.time() - self.receive_latency
                    }
                    self.ring_buffer.put(state)

                    # fetch command from queue
                    try:
                        commands = self.input_queue.get_all()
                        n_cmd = len(commands['cmd'])
                    except Empty:
                        n_cmd = 0
                    
                    # execute commands
                    for i in range(n_cmd):
                        command = dict()
                        for key, value in commands.items():
                            command[key] = value[i]
                        cmd = command['cmd']
                        
                        if cmd == Command.SHUTDOWN.value:
                            keep_running = False
                            # stop immediately, ignore later commands
                            break
                        elif cmd == Command.SCHEDULE_WAYPOINT.value:
                            target_pos = command['target_pos'] * self.scale
                            target_time = command['target_time']
                            # translate global time to monotonic time
                            target_time = time.monotonic() - time.time() + target_time
                            curr_time = t_now
                            pose_interp = pose_interp.schedule_waypoint(
                                pose=[target_pos, 0, 0, 0, 0, 0],
                                time=target_time,
                                max_pos_speed=self.move_max_speed,
                                max_rot_speed=self.move_max_speed,
                                curr_time=curr_time,
                                last_waypoint_time=last_waypoint_time
                            )
                            last_waypoint_time = target_time
                        elif cmd == Command.RESTART_PUT.value:
                            t_start = command['target_time'] - time.time() + time.monotonic()
                            iter_idx = 1
                        else:
                            keep_running = False
                            break
                        
                    # first loop successful, ready to receive command
                    if iter_idx == 0:
                        self.ready_event.set()
                    iter_idx += 1
                    
                    # regulate frequency
                    dt = 1 / self.frequency
                    t_end = t_start + dt * iter_idx
                    precise_wait(t_end=t_end, time_func=time.monotonic)
                
        finally:
            self.ready_event.set()
            if self.verbose:
                print(f"[WSGController] Disconnected from robot: {self.hostname}")