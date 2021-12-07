from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
from math import *
from geometry import *
from time import *
class MyRobot2(RCJSoccerRobot):
    def move(self, dest):
        robot_pos = self.data[self.name]
        dest_angle, robot_angle = self.get_angles(dest, robot_pos)
        k = dest_angle
        lv = 0
        rv = 0
        if(k > 180): k -= 360
        v = 10
        if(getDisgtance(robot_pos, dest) < 0.01):
            v = 0
        if(dest_angle > 90 and dest_angle < 270):
            if(k<0): k += 180
            else: k -= 180
            if dest_angle < 160:
                lv = 4
                rv = -4
            elif dest_angle > 200:
                lv = -4
                rv = 4
            else:
                lv = v - k/10
                rv = v + k/10
        else:
            if dest_angle > 20 and dest_angle < 180:
                lv = -4
                rv = 4
            elif dest_angle < 340 and dest_angle > 180:
                lv = 4
                rv = -4
            else:
                lv = -v - k/10
                rv = -v + k/10
        if(rv > 10): rv = 10
        if(lv > 10): lv = 10
        if(rv <-10): rv =-10
        if(lv <-10): lv =-10
        # self.left_motor.setVelocity(lv)
        # self.right_motor.setVelocity(rv)
    
    def run(self):
        ball2goal = Line()
        behindBall = Line()
        gaol_pos = {'x': 0.75, 'y': 0}
        goal_keeper_x = -0.67
        flag = False
        last_ball_pos = {'x': 0, 'y': 0}
        last_ball_time = 0
        cnt = 0
        sleep(0.1)
        self.role = 'forward'
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                self.data = self.get_new_data()
                ball_pos = self.data['ball']
                predicted_ball_pos = ball_pos
                robot_pos = self.data[self.name]
                distances = [
                    getDisgtance(ball_pos, self.data[self.name[0] + '1']),
                    getDisgtance(ball_pos, self.data[self.name[0] + '2']),
                    getDisgtance(ball_pos, self.data[self.name[0] + '3'])
                ]
                if(self.name[1] == str(distances.index(max(distances))+1)):
                    self.role = 'goalkeeper'
                else: 
                    self.role = 'forward'
                ball_speed = getDisgtance(ball_pos, last_ball_pos)/(time() - last_ball_time)

                if(self.role == 'forward'):
                    if(ball_speed > 0.1 and ball_speed < 5):
                        delta_x = ball_speed*10
                        angle = getAngle(ball_pos, last_ball_pos)
                        x = delta_x * cos(angle)
                        y = delta_x * sin(angle)
                        ball_pos['x'] += x
                        ball_pos['y'] += y
                    ball2goal.drawLineWithTwoPoint(ball_pos, gaol_pos)  
                    aroundBall = Circle(ball_pos, 0.1)
                    points = aroundBall.getIntersectionWithLine(ball2goal)
                    p = points[0]
                    if(len(points) > 1):
                        if(getDisgtance(gaol_pos, points[0]) < getDisgtance(gaol_pos, points[1])):
                            p = points[1]
                    if(getDisgtance(p, robot_pos) > 0.05 and not flag):
                        self.move(p)
                    else:
                        self.move(ball_pos)
                        flag = True
                        if(getDisgtance(p, robot_pos) > 0.15):
                            flag = False
                elif(self.role == 'goalkeeper'):
                    ball_direction_line = Line()
                    ball_direction_line.drawLineWithTwoPoint(ball_pos, last_ball_pos)
                    gaol_keeper_line = Line(0, 1, 0.67)
                    intersection = ball_direction_line.getIntersectionWithLine(gaol_keeper_line)
                    goal_keeper_y = ball_pos['y']
                    if(intersection != None):
                        goal_keeper_y = intersection['y']
                    if(goal_keeper_y > 0.4): goal_keeper_y = 0.4
                    if(goal_keeper_y <-0.4): goal_keeper_y =-0.4
                    self.move({'x': goal_keeper_x , 'y': goal_keeper_y})

                last_ball_pos = self.data['ball']
                last_ball_time = time()

