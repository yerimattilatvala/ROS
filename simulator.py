#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter and Mobile Robot Experiment
"""
import argparse
import struct
import sys
import copy
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header
from std_msgs.msg import Empty
from baxter_core_msgs.srv import SolvePositionIK
from baxter_core_msgs.srv import SolvePositionIKRequest
import baxter_interface
from baxter_interface import CHECK_VERSION
from random import uniform
from random import randint
from math import atan2
from math import sqrt
import numpy as np

LIMIT_DOWN_X = 0.4
LIMIT_UP_X = 0.6
LIMIT_DOWN_Y = -0.2
LIMIT_UP_Y = 0.3
CORR_X = 0.025
CORR_Y = 0.025
POZ_Z = -0.130
POS_Z = 0.9
PROX_X = 0.09
PROX_Y = 0.09
DIST = 0.05
MOBILE_POSITION_X_MIN = 0.4
MOBILE_POSITION_X_MAX = 0.75
MOBILE_POSITION_Y_MIN = -0.4
MOBILE_POSITION_Y_MAX = 0.4
ORIENTATION1 = Quaternion(x=0,
                        y=1,
                        z=0,
                        w=0)

BLOCK = 'block'
BOWL = 'bowl'
ROBOBO = 'mobile'
GOAL = 'goal'

class Simulator():

    def __init__(self, verbose = 1, hover_distance = 0.15):
        self._hover_distance = hover_distance
        self._left_arm = baxter_interface.limb.Limb("left")
        self._left_joint_names = self._left_arm.joint_names()
        self._left_gripper = baxter_interface.gripper.Gripper('left')
        self._rate = rospy.Rate(1)
        self._verbose = verbose
        self._subs = []
        ns = "ExternalTools/" + 'left' + "/PositionKinematicsNode/IKService"                             
        self._ros = rospy.ServiceProxy(ns, SolvePositionIK)                                              
        rospy.wait_for_service(ns, 5.0)  
        print("***Obteniendo estado del baxter... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("***Activando robot... ")
        self._rs.enable()
        self.set_neutral()
        self._left_gripper.calibrate()
        self.gripper_close()
        self._reached_position = False
        self._running = False
        self._found = False
        self._i = 0
        self._run = True
        self._rebooting = False
        self._block_pose = None
        self._bowl_pose = None
        self._get_poses = True
        self._grab = False
        self._robot_catch_block = False

    def _get_rotation(self, msg):
        self._mobile_position_x = msg.pose.pose.position.x
        self._mobile_position_y = msg.pose.pose.position.y
        mobile_robot_orientation = msg.pose.pose.orientation
        orientation_list = [mobile_robot_orientation.x, 
                            mobile_robot_orientation.y, 
                            mobile_robot_orientation.z, 
                            mobile_robot_orientation.w]
        (_, _, self._mobile_yaw) = euler_from_quaternion(orientation_list)

    def _publish_message(self, msg):
        if self._publisher is not None: 
            self._publisher.publish(msg)

    def _read_position(self, msg):
        pos = 0
        self._get_poses = False
        for x in msg.name:
            if x == BLOCK:
                pose = msg.pose[pos]
                print 'OBTENIENDO POSICION : CUBO ....'
                if self._i == 0:
                    block_x = pose.position.x + CORR_X
                    self._i +=1
                else:
                    block_x = pose.position.x - CORR_X
                block_y = pose.position.y + CORR_Y
                self._block_pose = Pose(position=Point(x=block_x, 
                                            y=block_y, 
                                            z=POZ_Z), 
                                            orientation=ORIENTATION1)
                print 'POSICION CUBO : ', self._block_pose 
            elif x == GOAL:
                pose = msg.pose[pos]
                print 'OBTENIENDO POSICION : GOAL....'
                bowl_x = pose.position.x
                bowl_y = pose.position.y
                bowl_z = POZ_Z
                self._bowl_pose = Pose(position=Point(x=bowl_x, 
                                            y=bowl_y, 
                                            z=bowl_z), 
                                            orientation=ORIENTATION1)               
                print 'POSICION BOWL : ', self._bowl_pose
            pos += 1
        #rospy.loginfo("The ball is in the position : x:{}, y:{}, z:{}".format(self._pos_x, self._pos_y, self._pos_z))

    def _unregister_all_subscriber(self):
        for publisher in self._subs:
            publisher.unregister()

    def _unregister_all(self):
        self._unregister_all_subscriber()
        self._publisher.unregister()

    def _reset_control_modes(self):
        rate = rospy.Rate(self._rate)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._left_arm.exit_control_mode()
            rate.sleep()
        return True
    
    def _reset_models(self):
        model_state = ModelState()
        model_state_bowl = ModelState()
        model_state_bowl.model_name = GOAL
        model_state.model_name = BLOCK
        model_state_bowl.pose.position.x = self._bowl_pose.position.x
        model_state_bowl.pose.position.y = self._bowl_pose.position.y
        model_state_bowl.pose.position.z = 0.1
        model_state_bowl.reference_frame = 'base'
        print 'RESET BLOCK POSE...'
        model_state.pose.position.x = round(uniform(LIMIT_DOWN_X, LIMIT_UP_X-0.05), 4) - CORR_X
        model_state.pose.position.y = round(uniform(LIMIT_DOWN_Y, LIMIT_UP_Y-0.2), 4)
        while(np.abs(model_state.pose.position.y - self._bowl_pose.position.y)< 0.15):
            model_state.pose.position.y = round(uniform(LIMIT_DOWN_Y, LIMIT_UP_Y-0.2), 4)
        model_state.pose.position.z = POS_Z
        model_state.pose.orientation = ORIENTATION1
        model_state.reference_frame = 'base'
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            sms = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            print 'Model State: ' + str(model_state)
            sms(model_state)
            sms(model_state_bowl)
            print 'BLOCK ORIENTATION : 0K'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        rospy.sleep(2.0)

    def set_neutral(self):
        print("***Colocando baxter en posicion de inicio...")
        self._left_arm.move_to_neutral()

    def clean_shutdown(self):
        print("\n***Terminando ejecucion...")
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("***Desactivando robot...")
            self._rs.disable()
        return True

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._ros(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            #rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            raise Exception('INVALID POSE - No Valid Joint Solution Found.')
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._left_arm.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        print '**Abriendo gripper....'
        self._left_gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        print '***Cerrando gripper...'
        self._left_gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        print '***Situandose sobre posicion...', pose
        approach = copy.deepcopy(pose)
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        print '***Subiendo brazo...'
        # retrieve current pose from endpoint
        current_pose = self._left_arm.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        print '***Bajando brazo...'
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

    def _reboot(self):
        if self._rebooting:
            self.set_neutral()
            self._reset_models()
            rospy.sleep(2.0)
            self._found = False
            self._run = True
            self._rebooting = False
            self._block_pose = None
            self._bowl_pose = None
            self._get_poses = True
            self._grab = False
            self.gripper_close()
            rospy.sleep(3.0)         

    def _moveIt(self):
        try:            
            if self._found:
                self._found = False
                if self._grab == False:
                    if self._block_pose is not None:
                        self._approach(self._block_pose)
                        self.gripper_open()
                        self._servo_to_pose(self._block_pose)
                        self.gripper_close()
                        self._retract()
                        self._grab = True
                else:
                    if self._bowl_pose is not None:
                        self._approach(self._bowl_pose)
                        self._grab = False
                        self._rebooting = True
                        self.gripper_open()

        except Exception as _:
            print '***ERROR -> CALCULANDO SIGUIENTE POSE...'
        
    def _mobile_near_block(self):
        near = False
        distance_to_block_x = abs(self._block_pose.position.x - self._mobile_position_x)
        distance_to_block_y = abs(self._block_pose.position.y - self._mobile_position_y)
        distance = sqrt((distance_to_block_x**2) + (distance_to_block_y**2))
        if distance < 0.15:
            near = True
        return near

    def is_over(self, pose):
        over = False
        if self._block_pose is not None and self._bowl_pose is not None:
            if self._grab:
                print 'BOWL POSE : ', self._bowl_pose
                x1 = self._bowl_pose.position.x
                y1 = self._bowl_pose.position.y
            else:
                print '---BLOCK POSE : ', self._block_pose
                x1 = self._block_pose.position.x
                y1 = self._block_pose.position.y
            x2 = pose.position.x
            y2 = pose.position.y
            x = np.abs(x1 - x2)
            y = np.abs(y1 - y2)
            distance = np.sqrt(x**2 + y**2)
            print 'X : {0}, Y : {1}'.format(x,y)
            print 'DISTANCE : ', distance
            if x <= PROX_X and y <= PROX_Y:
                over = True
            #if distance <= DIST:
        return over

    def _generate_baxter_poses(self):
        x = round(uniform(LIMIT_DOWN_X, LIMIT_UP_X), 3)
        y = round(uniform(LIMIT_DOWN_Y, LIMIT_UP_Y), 3)
        pose =  Pose(position=Point(x=x, 
                                    y=y, 
                                    z=POZ_Z), 
                                    orientation=ORIENTATION1)
        return pose

    def _add_subscriber(self, subscriber):
        self._subs.append(subscriber)

    def _run_subscribers(self): 
        sub1 = rospy.Subscriber("/gazebo/model_states", ModelStates, self._read_position)
        sub2 = rospy.Subscriber('/odom', Odometry, self._get_rotation)
        self._add_subscriber(sub1)
        self._add_subscriber(sub2)

    def _run_publisher(self):
        self._publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def _generate_random_pose_to_mobile_robot(self):
        self._mobile_target_position_x = uniform(MOBILE_POSITION_X_MIN, MOBILE_POSITION_X_MAX)
        self._mobile_target_position_y = uniform(MOBILE_POSITION_Y_MIN, MOBILE_POSITION_Y_MAX)
        
    def _go_forward(self, potence=0.05):
        print 'Moviendose en linea recta...'
        msg = Twist()
        msg.linear.x = potence
        msg.angular.z = 0.0
        self._publish_message(msg)
        print 'Movido'

    def _stop(self):
        print 'Stopping ...'
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self._publish_message(msg)
        print 'Stop'

    def _rotate(self, potence=0.05):
        print 'Rotating...'
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = potence
        self._publish_message(msg)

    def _angle_beetwen_2_points(self):
        x = self._mobile_target_position_x - self._mobile_position_x
        y = self._mobile_target_position_y - self._mobile_position_y
        self._angle_target = atan2(y, x)

    def _block_on_robot(self):
        model_state = ModelState()
        model_state.model_name = BLOCK
        print 'MOVE BLOCK...'
        model_state.pose.position.x = self._mobile_position_x
        model_state.pose.position.y = self._mobile_position_y
        model_state.pose.position.z = POS_Z
        model_state.pose.orientation = ORIENTATION1
        model_state.reference_frame = 'base'
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            sms = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            sms(model_state)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def _move_mobile_robot_randomly(self):
        if self._running:
            self._generate_random_pose_to_mobile_robot()
            is_rotating = False
            run = False
            is_runing = False
            while not self._reached_position:
                print 'CURRENT POSITION : ({0},{1}) || TARGET : ({2},{3})'.format(self._mobile_position_x,
                                                                                self._mobile_position_y,
                                                                                self._mobile_target_position_x,
                                                                                self._mobile_target_position_y)
                if is_rotating == False:
                    if self._mobile_target_position_y > 0:
                        self._rotate(potence=-0.1)
                    else:
                        self._rotate(potence=0.1)
                    is_rotating = True
                if run:
                    if is_runing == False:
                        self._go_forward()
                        is_runing = True
                    else:
                        position_x = abs(self._mobile_target_position_x - self._mobile_position_x)
                        position_y = abs(self._mobile_target_position_y - self._mobile_position_y)
                        print 'DISTANCES : X : {0}, Y : {1}'.format(position_x, position_y)
                        if self._robot_catch_block:
                            if (position_x < 0.1 and position_y < 0.1):
                                self._stop()
                                print 'PUNTO OBJETIVO ALCANZADO'
                        else:
                            if (position_x < 0.1 and position_y < 0.1) or self._mobile_near_block():
                                self._stop()
                                print 'PUNTO OBJETIVO ALCANZADO'
                                if self._mobile_near_block():
                                    self._block_on_robot()
                                    self._robot_catch_block = True
                            
                        self._reached_position = True
                else:
                    self._angle_beetwen_2_points()
                    if abs(self._angle_target - self._mobile_yaw) < 0.2:
                        self._stop()
                        run = True
                self._rate.sleep()
            self._reached_position = False

    def run(self):
        self._run_subscribers()
        self._run_publisher()
        rospy.sleep(2)
        print '***Pulsa Ctrl-C para parar...'
        while not rospy.is_shutdown():
            self._running = True
            self._move_mobile_robot_randomly()
            '''if self._run:
                pose = self._generate_baxter_poses()
                self._approach(pose)

                if self.is_over(pose) == True:
                    self._found = True

                self._moveIt()
                self._reboot()'''

    def delete_gazebo_models(self):
        try:
            self._unregister_all()
            self._run = False
            self._block_pose = None
            self._block_pose = None
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            delete_model("table")
            delete_model("block")
            delete_model(GOAL)
            delete_model("mobile")
            self.clean_shutdown()
            rospy.sleep(1.0)
        except rospy.ServiceException, e:
            rospy.loginfo("Delete Model service call failed: {0}".format(e))

def load_gazebo_models( table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.6,y=0.1, z=0.1)),
                       block_reference_frame="base"):
    p0 = Pose(position=Point(x=0.6, y=0.0, z=-0.2), orientation=Quaternion(x=0,y=0,z=-1,w=1))
    p1 = Pose(position=Point(x=0.5, y=0.2, z=0.1))
    p2 = Pose(position=Point(x=0.5, y=-0.3, z=0.1))
    print '***Cargando modelos.....'
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('simulator')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml = ''
    with open (model_path + "block/model.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
    goal_xml = ''
    with open (model_path + "goal/model.urdf", "r") as bowl_file:
        goal_xml=bowl_file.read().replace('\n', '')
    mobile_robot_xml = ''
    with open (model_path + "my_robot1/model.sdf", "r") as mobile_robot_file:
        mobile_robot_xml=mobile_robot_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:      
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_sdf("table", table_xml, "/",p0, table_reference_frame)
        #spawn_sdf("bowl", bowl_xml, "/",p1, "base")
        spawn_sdf("mobile", mobile_robot_xml, "/",p2, "base")
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
        spawn_urdf(GOAL, goal_xml, "/", p1, "base")
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    print '***Entorno cargado.'
    rospy.sleep(1.0)


def main():
    print "***Inicializando nodo... " 
    rospy.init_node("baxterSimulatorExperiment")
    sim = Simulator(verbose=0)
    load_gazebo_models()
    rospy.on_shutdown(sim.delete_gazebo_models)
    sim.run()
    print "***Terminado." 

if __name__ == '__main__':
    main()