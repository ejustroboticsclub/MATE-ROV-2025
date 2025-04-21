import pygame
import concurrent.futures as ftu
from queue import Queue
import rclpy
from rclpy.node import Node
#TODO: set this import to package name
from rov25 import live_queue_plotter as que
from std_msgs.msg import Float64, Float32MultiArray
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R

max_kp = 10
min_kp = 0

action_queue = Queue()

def pygame_loop():
    pygame.init()
    screen = pygame.display.set_mode((960, 540))
    pygame.display.set_caption("Calibaration Node")
    running = True
    font = pygame.font.SysFont("Arial", 18)
    input_mode = False
    single = True
    setting = False
    edit = 0
    input_text = ""
    errors = ""
    clock = pygame.Clock()
    clock.tick(60)
    desired = -1000
    bool_desired = False
    constants = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    current = 0
    
    
    while running:
        if not input_mode:
            screen.fill((4,20,33))
        else:
            screen.fill((40,4,20))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif not input_mode:
                    if event.key == pygame.K_c:
                        input_mode = True
                        single = True
                        setting = False
                        input_text = ""
                        desired = -1000
                        edit = 2
                    elif event.key == pygame.K_a:
                        input_mode = True
                        single = False
                        input_text = ""
                        edit = 1
                    elif event.key == pygame.K_q:
                        input_mode = True
                        single = True
                        setting = True
                        input_text = ""
                        edit = 1
                    elif event.key == pygame.K_d:
                        input_mode = True
                        single = True
                        setting = True
                        bool_desired = True
                        input_text = ""
                        edit = 3
                    elif event.key == pygame.K_UP:
                        constants[current] += 0.1
                        constants[current] = min(constants[current], max_kp)
                        constants[current] = round(constants[current], 3)
                        edit = 1
                        action_queue.put((edit, current, constants))
                        edit = 0
                    elif event.key == pygame.K_DOWN:
                        constants[current] -= 0.1
                        constants[current] = max(constants[current], min_kp)
                        constants[current] = round(constants[current], 3)
                        edit = 1
                        action_queue.put((edit, current, constants))
                        edit = 0
                    elif event.key == pygame.K_w:
                        constants[current] += 0.5
                        constants[current] = min(constants[current], max_kp)
                        constants[current] = round(constants[current], 3)
                        edit = 1
                        action_queue.put((edit, current, constants))
                        edit = 0
                    elif event.key == pygame.K_s:
                        constants[current] -= 0.5
                        constants[current] = max(constants[current], min_kp)
                        constants[current] = round(constants[current], 3)
                        edit = 1
                        action_queue.put((edit, current, constants))
                        edit = 0
                else:
                    if event.key == pygame.K_RETURN:
                        input_mode = False
                        if not single:
                            if len(input_text.split(" ")) == 6:
                                try:
                                    constants= list(map(float, input_text.split(" ")))
                                    action_queue.put((edit, current, constants))
                                except ValueError:
                                    errors = "Enter a valid number"
                            else:
                                errors = "You must enter all parameters in this mode"
                        elif setting and not bool_desired:
                            if len(input_text.split(" ")) == 1:
                                try:
                                    constants[current] = float(input_text.strip())
                                    action_queue.put((edit, current, constants))
                                except ValueError:
                                    errors = "Enter a valid number"
                            else:
                                errors = "You must enter a single parameters in this mode"
                        elif setting:
                            if len(input_text.split(" ")) == 1:
                                try:
                                    desired = float(input_text.strip())
                                    action_queue.put((edit, current, desired))
                                    bool_desired = False
                                except ValueError:
                                    errors = "Enter a valid number"
                            else:
                                errors = "You must enter a single parameters in this mode"
                        else:
                            if len(input_text.split(" ")) == 1:
                                try:
                                    temp = int(input_text.strip())
                                    if 0<=temp<6:
                                        current = temp
                                        action_queue.put((edit, current, constants))
                                    else:
                                        errors = "Out of range index. Must be 0-5"
                                except ValueError:
                                    errors = "Enter a valid number"
                            else:
                                errors = "You must enter a single parameters in this mode"
                        input_text = ""
                        edit = 0    
                    elif event.key == pygame.K_BACKSPACE:
                        input_text = input_text[:-1]
                    else:
                        if event.unicode.isprintable():
                            input_text += event.unicode
                    
        text_surface_line1 = font.render("Choose (c), All(a), Set(q), set desired(d),+/-0.1(up/down), +/-0.5(w/s)", True, (208,214,214))
        text_surface_line2 = font.render(f"""kp_x: {constants[0]}, kp_y: {constants[1]}, kp_wz: {constants[2]}, kp_depth: {constants[3]}, kp_roll: {constants[4]}, kp_pitch: {constants[5]}"""
                                         , True, (208,214,214))
        text_surface_line3 = font.render(f"Currently controling <<< index: {current} >>> desired: {desired}", True, (208, 214, 214))
        text_surface_line4 = font.render("input space seperated:---> " + input_text, True, (208, 214, 214))
        text_surface_line5 = font.render(errors, True, (245,25,48))
        
        screen.blit(text_surface_line1, (0,0))
        screen.blit(text_surface_line2, (0, 18))
        screen.blit(text_surface_line3, (0,36))
        screen.blit(text_surface_line4, (0, 54))
        screen.blit(text_surface_line5, (0, 72))
        
        pygame.display.flip()

        
class Calibration(Node):
    def __init__(self):
        super().__init__("Calibration")
        self.depth_subscribtion = self.create_subscription(Float64, "ROV/depth", self.depth_received_callback, 10)
        self.imu_subscribtion = self.create_subscription(Imu, "ROV/imu", self.imu_received_callback, 10)
        self.depth_subscribtion
        self.imu_subscribtion
        self.constants_publisher = self.create_publisher(Float32MultiArray, "ROV/constants", 10)
        self.desired_publisher = self.create_publisher(Float32MultiArray, "ROV/desired", 10)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.spawn_plots()
        self.constants = []
        self.current = 0
    
    #TODO: correctly set the queue size and duration
    def spawn_plots(self):
        que.initialize_queue(1000, 10)
        que.draw()
    
    def timer_callback(self):
        while not action_queue.empty():
            temp = action_queue.get()
            print(temp)
            if temp[0] == 1:
                self.constants = temp[2]
                msg = Float32MultiArray()
                msg.data = self.constants
                self.constants_publisher.publish(msg)
            elif temp[0] == 2:
                self.current = temp[1]
                self.spawn_plots()
            else:
                desired = [-1000.0]*6
                for i in range(6):
                    if i == self.current:
                        desired[i] = temp[2]
                    else:
                        desired[i] = -1000.0
                msg = Float32MultiArray()
                msg.data = desired
                self.desired_publisher.publish(msg)
        que.draw()
        
        
            
    def depth_received_callback(self, msg: Float64):
        new_point = msg.data
        if self.current == 3:
            que.update(new_point)
            
    
    def imu_received_callback(self, msg: Imu):
        q = msg.orientation
        q = [q.x, q.y, q.z, q.w]
        r = R.from_quat(q)
        orien = r.as_euler("xyz", degrees=False)
        if self.current == 2:
            new_point = orien[2]
            que.update(new_point)
        elif self.current == 4:
            new_point = orien[0]
            que.update(new_point)
        elif self.current == 5:
            new_point = orien[1]
            que.update(new_point)
  
        
def main(args = None):
    rclpy.init(args=args)
    cal_node = Calibration()
    executer = ftu.ThreadPoolExecutor(max_workers = 1)
    try:
        executer.submit(pygame_loop)
        rclpy.spin(cal_node)
        executer.shutdown(wait=True)
    except KeyboardInterrupt:
        print("Shutting down ...")
    finally:
        pygame.quit()
        cal_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
