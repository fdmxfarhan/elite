from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
from math import *
class MyRobot2(RCJSoccerRobot):
    def move(self, dest):
        dest_angle, robot_angle = self.get_angles(dest, self.data[self.name])
        
        if dest_angle > 15 and dest_angle < 180:
            self.left_motor.setVelocity(-4)
            self.right_motor.setVelocity(4)
        elif dest_angle < 345 and dest_angle > 180:
            self.left_motor.setVelocity(4)
            self.right_motor.setVelocity(-4)
        else:
            self.left_motor.setVelocity(-10)
            self.right_motor.setVelocity(-10)
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                self.data = self.get_new_data()
                ball_pos = self.data['ball']
                robot_pos = self.data[self.name]
                
                # if(abs(ball_pos['x'] - robot_pos['x']) > 0.1 or abs(ball_pos['y'] - robot_pos['y']) > 0.1):
                #     self.move({'x': ball_pos['x'] - 0.1, 'y': ball_pos['y']})
                # else:
                #     self.move(ball_pos)

