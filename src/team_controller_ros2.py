import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
import math
import numpy as np
from simplex import *
import threading

from epuck_driver_interfaces.action import SimpleMovement
from rclpy.action import ActionClient

def vector_from_angle(ori):
     return np.array([math.cos(ori),math.sin(ori)])


def get_triangle_positions(element_distance):
        
        # Coordinates are relative anyway
        ori = 0.0
        center = np.array([0., 0.])

        directions = np.concatenate((vector_from_angle(ori).reshape(-1, 2),
                                     vector_from_angle(ori - 2*math.pi/3).reshape(-1, 2),
                                     vector_from_angle(ori + 2*math.pi/3).reshape(-1, 2)), axis=0)
        
        a = element_distance/(2 * math.sin(math.pi/3))
        return directions * a + center

def send_movement_goal(client, angle, distance):

    goal_msg = SimpleMovement.Goal()
    goal_msg.angle = angle
    goal_msg.distance = distance

    if not client.wait_for_server(5):
        raise Exception("Timeout on movement control action server")
    
    return client.send_goal_async()


class TeamController(Node):

    def __init__(self):
        super().__init__('TeamController')
        
        
        self.robot_ids = [0, 1, 2]
        self.robot_distance = 0.3  # in meters
        self.record_time = 3  # in seconds
        self.step_distance = 0.1  # in meter
        self.prev_mov_dir = None
    
        self.microphone_buffers = {id:{k:[] for k in range(3)} for id in self.robot_ids}
        self.microphone_data_subscribers = [[self.create_subscription(Float32, 
                                                                      "{}/microphone_{}".format(id, k), 
                                                                      self.create_microphone_data_callback(id, k)) for k in range(3)] for id in self.robot_ids]
        self.robot_move_command_publishers = [self.create_publisher() for id in self.robot_ids]

        self.is_converged = False

        self.sound_localization_done = threading.Event()
        self.formation_translation_done = threading.Event()

        self.movement_controller_action_clients = [ActionClient(SimpleMovement, "{}/movement_goal".format(id)) for id in self.robot_ids]




    def create_microphone_data_callback(self, robot_id, mic_id):
         
        def callback(data):
            if self.record:
                self.microphone_buffers[robot_id][mic_id].append(data)
        
        return callback
    
    def clear_buffers(self):
         
         for id in self.robot_ids:
              for k in range(3):
                   self.microphone_buffers[id][k].clear()

    def init_sound_localization(self):

        self.clear_buffers() 
        self.sound_collection_done.clear()
        self.record = True

        self.timer = self.create_timer(self.record_time, self.sound_localization())


    def issue_triangle_translation(self, direction):

        self.formation_translation_done.clear()
        d_theta = math.atan2(direction[1], direction[0])

        futures = [send_movement_goal(action_client, d_theta, self.step_distance) for action_client in self.movement_controller_action_clients]
        for future, action_client in zip(futures, self.movement_controller_action_clients):
            rclpy.spin_until_future_complete(action_client, future)


    # using averaging strategy for now
    def sound_localization(self):
        
        self.record = False
        
        time_avg_dBs = np.array([[np.average(np.abs(self.microphone_buffers[id][k])) for k in range(3)] for id in self.robot_ids])
        robot_avg_dBs = np.average(time_avg_dBs.reshape(-1, 3), axis=1)
        robot_locs = get_triangle_positions(self.robot_distance)
        microphone_data = [{"pos":robot_locs[i], "dB":robot_avg_dBs[i]} for i in range(len(robot_avg_dBs))]

        separations = determine_sound_source_halfspaces(microphone_data, True)
        simplex = aggregate_separations_to_simplex(separations)
        mov_dir, sampled_outer_points = determine_direction(simplex, 
                                                            np.array([0., 0.]), 
                                                            self.robot_distance)
        # check if more than 90 degree direction change
        if self.prev_mov_dir is not None:
            if self.prev_mov_dir @ mov_dir / (np.linalg.norm(self.prev_mov_dir) * np.linalg.norm(self.prev_mov_dir)) < 0:
                # convergence
                self.is_converged = True
        self.prev_mov_dir = mov_dir

        self.sound_localization_done.set()



    def main_control_loop(self, rate):

        # 1. Stage: Optimize Formation
        # 2. Stage: Do soundlocalizaton and check convergence
        # 3. Stage: translate Formation
        
                           #create_stage_executer(, self.formation_translation_done, rate)]

        while not self.is_converged:
            
            self.init_sound_localization()

            while not self.sound_localization_done and rclpy.ok():
                rate.sleep()
            
            if not rclpy.ok():
                break

            self.issue_triangle_translation()


             
             
def main(args=None):
    rclpy.init(args=args)

    team_controller = TeamController()

    #rclpy.spin(TeamController)

    thread = threading.Thread(target=rclpy.spin, args=(team_controller, ), daemon=True)
    thread.start()

    rate = team_controller.create_rate(10)
    try:
        team_controller.main_control_loop(rate)
    except KeyboardInterrupt:
        pass
   

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    thread.join()
    thread.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()