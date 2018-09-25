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
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from gazebo_msgs.msg import ModelStates

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
from baxter_interface import CHECK_VERSION

from random import uniform

LIMIT_DOWN_X = 0.3
LIMIT_UP_X = 0.6
LIMIT_DOWN_Y = -0.7
LIMIT_UP_Y = 0.7
POZ_Z = -0.129
ORIENTATION = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)

class Simulator():

    def __init__(self, object_name, hover_distance = 0.15):
        self._hover_distance = hover_distance
        self._object_name = object_name
        self._left_arm = baxter_interface.limb.Limb("left")
        #self._right_arm = baxter_interface.limb.Limb("right")
        self._left_joint_names = self._left_arm.joint_names()
        #self._right_joint_names = self._right_arm.joint_names()
        self._left_gripper = baxter_interface.gripper.Gripper('left')
        #self._right_gripper = baxter_interface.gripper.Gripper('right')
        # control parameters
        self._rate = 500.0  # Hz
        self._found = False
        self._pose = None
        ns = "ExternalTools/" + 'left' + "/PositionKinematicsNode/IKService"                              # servicio de ros a ejecutar
        self._ros = rospy.ServiceProxy(ns, SolvePositionIK)                                               # manejador del servicio ns
        rospy.wait_for_service(ns, 5.0)  
        print("Obteniendo estado del baxter... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Activando robot... ")
        self._rs.enable()
        self.set_neutral()

    def _callback(self, msg):
        pos = 0
        catch = False
        for x in msg.name:
            if x == self._object_name:
                catch = True
                break
            pos += 1
        #rospy.loginfo(msg.pose[pos])
        #self._pose = msg.pose[pos]
        if (self._pose is None or self._found) and catch == True:
            pose = msg.pose[pos]
            self._pos_x = pose.position.x
            self._pos_y = pose.position.y
            self._pos_z = pose.position.z
            self._pose = Pose(position=Point(x=self._pos_x, 
                                                    y=self._pos_y, 
                                                    z=POZ_Z), 
                                                    orientation=ORIENTATION)
        else:
            pose = msg.pose[pos]
            pos_z = self._pos_z - 0.1
            if pose.position.z < pos_z:
                self.set_neutral()
                delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)      
                delete_model("block")
                # Get Models' Path
                model_path = rospkg.RosPack().get_path('simulator')+"/models/"
                block_pose=Pose(position=Point(x=0.6725, y=0.1265, z=1.7825))
                block_reference_frame="world"
                # Load Block URDF
                block_xml = ''
                with open (model_path + "block/model.urdf", "r") as block_file:
                    block_xml=block_file.read().replace('\n', '')
                # Spawn Block URDF
                rospy.wait_for_service('/gazebo/spawn_urdf_model')
                try:
                    spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
                    spawn_urdf("block", block_xml, "/",block_pose, block_reference_frame)
                except rospy.ServiceException, e:
                    rospy.logerr("Spawn URDF service call failed: {0}".format(e))
                
                rospy.sleep(1)


        #rospy.loginfo("The ball is in the position : x:{}, y:{}, z:{}".format(self._pos_x, self._pos_y, self._pos_z))

    def _reset_control_modes(self):
        rate = rospy.Rate(self._rate)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._left_arm.exit_control_mode()
            #self._right_arm.exit_control_mode()
            rate.sleep()
        return True
    
    def set_neutral(self):
        print("***Colocando baxter en posicion de inicio...")
        self._left_arm.move_to_neutral()
        #self._right_arm.move_to_neutral()

    def clean_shutdown(self):
        print("\n***Terminando ejecucion...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("***Desactivando robot...")
            self._rs.disable()
        return True

    def ik_request(self, pose):
        # Recobe un objeto Pose con la posicion y orientacion del objeto
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._ros(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                        (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            
            print("IK Joint Solution:\n{0}".format(limb_joints))
            print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
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
        # approach with a pose the hover-distance above the requested pose
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

    def moveIt(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)

    def _generate_baxter_poses(self):
        x = round(uniform(LIMIT_DOWN_X, LIMIT_UP_X), 3)
        y = round(uniform(LIMIT_DOWN_Y, LIMIT_UP_Y), 3)
        pose =  Pose(position=Point(x=x, 
                                    y=y, 
                                    z=POZ_Z), 
                                    orientation=ORIENTATION)
        return pose

    def run(self,block_poses):
        self._subs = rospy.Subscriber("/gazebo/model_states", ModelStates, self._callback,queue_size=1)               # para obtener la posicion del objeto a buscar
        idx = 0
        print '***Pulsa Ctrl-C para parar...'
        while not rospy.is_shutdown():
                #print self._pose
                #print self._subs.queue_size
                self.moveIt(self._generate_baxter_poses())
                #print("\nPicking...")
                #self.pick(block_poses[idx])
                ''' print("\nPlacing...")
                idx = (idx+1) % len(block_poses)
                self.place(block_poses[idx])'''

    def delete_gazebo_models(self):
        # This will be called on ROS Exit, deleting Gazebo models
        # Do not wait for the Gazebo Delete Model service, since
        # Gazebo should already be running. If the service is not
        # available since Gazebo has been killed, it is fine to error out
        try:
            self._subs.unregister()
            self._pose = None
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            delete_model("table")
            delete_model("block")
            self.clean_shutdown()
        except rospy.ServiceException, e:
            rospy.loginfo("Delete Model service call failed: {0}".format(e))

def load_gazebo_models( table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.3,y=-0.5, z=1.7825)),
                       block_reference_frame="world"):
    p0 = Pose(position=Point(x=0.6, y=0.0, z=-0.2), orientation=Quaternion(x=0,y=0,z=-1,w=1))
    print '***Cargando modelos.....'
    '''l = []
    p0 = Pose(position=Point(x=0.6, y=0.0, z=-0.2), orientation=Quaternion(x=0,y=0,z=-1,w=1))
    p1 = Pose(position=Point(x=0.6, y=1.5, z=-0.2), orientation=Quaternion(x=0,y=0,z=-1,w=1))
    p2 = Pose(position=Point(x=1.39, y=0.0, z=-0.2), orientation=Quaternion(x=0,y=0,z=-1,w=1))
    p3 = Pose(position=Point(x=1.39, y=1.5, z=-0.2), orientation=Quaternion(x=0,y=0,z=-1,w=1))
    p4 = Pose(position=Point(x=1.39, y=-1.5, z=-0.2), orientation=Quaternion(x=0,y=0,z=-1,w=1))
    p5 = Pose(position=Point(x=0.6, y=-1.5, z=-0.2), orientation=Quaternion(x=0,y=0,z=-1,w=1))
    l.append(p0)
    l.append(p1)
    l.append(p2)
    l.append(p3)
    l.append(p4)
    l.append(p5)'''
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
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:      
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("table", table_xml, "/",p0, table_reference_frame)
        '''for i in range(len(l)):
            resp_sdf = spawn_sdf("table"+str(i), table_xml, "/",l[i], table_reference_frame)'''
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    print '***Entorno cargado.'
    rospy.sleep(1)


def main():
    print "***Inicializando nodo... " 
    rospy.init_node("rsdk_joint_velocity_wobbler")
    sim = Simulator('block')
    load_gazebo_models()
    rospy.on_shutdown(sim.delete_gazebo_models)
    # An orientation for gripper fingers to be overhead and parallel to the obj

    block_poses = list()
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    block_poses.append(Pose(position=Point(x=0.7, y=0.40, z=-0.129),
                            orientation=ORIENTATION))
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    block_poses.append(Pose(position=Point(x=0.75, y=0.0, z=-0.129),
                            orientation=ORIENTATION))
    sim.run(block_poses)

    print "-> Terminado." 

if __name__ == '__main__':
    main()