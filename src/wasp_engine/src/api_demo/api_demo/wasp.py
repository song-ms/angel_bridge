import os
from time import sleep
import curses
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



sub_topic_name = "/eth_bridge/joint_state_topic"
pub_topic_name = "/eth_bridge/joint_target_topic"


joint_names = ["l_hip", "l_knee", "r_hip", "r_knee"]

JOINT_STATE = 0

class WASP:
    def __init__(self, app_name):
        rclpy.init()
        self.node_ = Node(app_name)
        self.pub_ = self.node_.create_publisher(JointTrajectory, pub_topic_name, 10)
        self.sub_ = self.node_.create_subscription(JointTrajectory, sub_topic_name, self.SubsCallback, 10)

        self.state_ = JointTrajectory()
        state_pt = JointTrajectoryPoint()
        state_pt.positions = [0.0, 0.0, 0.0 ,0.0]
        self.state_.joint_names = joint_names
        self.state_.points.append(state_pt)

        self.command_ = JointTrajectory()
        commnad_pt = JointTrajectoryPoint()
        commnad_pt.positions = [0.0, 0.0, 0.0 ,0.0]
        commnad_pt.effort = [0.0, 0.0, 0.0 ,0.0]
        self.command_.joint_names = joint_names
        self.command_.points.append(commnad_pt)

        self.traj_ = []

        self.pos_ = [0.0, 0.0, 0.0 ,0.0]
        self.node_.get_logger().info('Start!')

    def setPosition(self, pos):
        for i, p in enumerate(pos):
            self.command_.points[0].positions[i] = p
        self.pub_.publish(self.command_)
    
    def setEffort(self, effort):
        for i, eff in enumerate(effort):
            self.command_.points[0].effort[i] = eff
        self.pub_.publish(self.command_)

    def getPosition(self):
        rclpy.spin_once(self.node_)
        return self.pos_

    def setTraj(self, traj):
        self.traj_ = traj
    
    def LaunchTraj(self):
        for i in range(len(self.traj_)):
            set_pos = self.traj_[i]
            self.setPosition(set_pos)
            cur_pos = self.getPosition()
            for j in range(4):
                self.node_.get_logger().info(f"{joint_names[j]}: state = {cur_pos[j]} , cmd = {set_pos[j]}")
            self.node_.get_logger().info("------------------------------------------------------------")
            sleep(0.01)

    def Terminate(self):
        self.setPosition([0, 0, 0, 0])
        self.node_.get_logger().info('Shutting down!')
        rclpy.shutdown()

    def Release(self):
        self.setEffort([-0, 0, 0, -0])
        sleep(1)

    def SubsCallback(self, msg):
        self.pos_ = msg.points[0].positions

    def uploadData(self, dummy):
        pass



def getTrajFromCSV(path):
    traj = []
    with open(path, "r") as f:
        for line in f.readlines() :
            tmp = line.rstrip('\n').split(',')
            traj.append([int(data) for data in tmp])
    return traj


def PrintPosition(pos):
    for j in range(4):
        print(f"{joint_names[j]}:\tposition = {pos[j]:.2f}")
    print("--------------------------------------------------")


class KeyOps:
    def __enter__(self):
        os.system('clear')
        self.screen_ = curses.initscr() # get the curses screen window
        curses.noecho()                 # turn off input echoing    
        curses.cbreak()                 # respond to keys immediately (don't wait for enter)        
        self.screen_.keypad(True)       # map arrow keys to special values
        self.screen_.addstr(0,0,'Keyops!')
        self.screen_.timeout(100)
        self.screen_.refresh()
        self.act = [0, 0, 0, 0]
        self.max = [2000, 1000, 2000, 1000]
        self.esc = False
        return self

    def getKeyInput(self):
        self.screen_.addstr(1,0,'TestFnc')
        key = self.screen_.getch()

        if key == 27: #esc
            self.esc = True
            return [0, 0, 0, 0]

        for i, ch in enumerate([113, 119, 111, 112]):
            if key == ch:
                self.act[i] += 300
            else:
                self.act[i] -= 100

            if self.act[i] > self.max[i]:
                self.act[i] = self.max[i]
            
            if self.act[i] < 0:
                self.act[i] = 0
        
        return [-self.act[0], self.act[1], self.act[2], -self.act[3]]

    def __exit__(self, exc_type, exc_val, exc_tb):
        curses.nocbreak()
        self.screen_.keypad(False)
        curses.echo()
        curses.endwin() # restore terminal