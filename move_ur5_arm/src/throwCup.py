#! /usr/bin/env python

import rospy
import os, sys 
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg

presentCase = 0
#startJointValues = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#prevJointValues = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#jointValues = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

'''
    the joint values are based on the following values:
    Initial is the initial pose, pose 1 = ready to grab the cup, pose 2 = grabbed cup and above pickup
    pose 3 = shake position 1, pose 4 = shake position 2 (repeated 5 times)
    pose 5 = Ready to throw, pose 6 = End pose of throw, pose 7 = pose 1

    Joints: elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint

    Initial:    -2.2029340902911585     =   -2.2030
                -1.6153925100909632     =   -1.6154
                -2.0307639280902308     =   -2.0308
                -3.017985169087545      =   -3.0180
                -1.4572399298297327     =   -1.4572
                -1.5013311545001429     =   -1.5302        1.5013

    Pose 1:     -2.191878143941061      =   -2.1919
                -1.8773024717914026     =   -1.8773
                -1.876983944569723      =   -1.8770
                -2.910978142415182      =   -2.9110
                -1.4573000113116663     =   -1.4573
                -1.501343075429098      =   -1.5302         1.5013
    
    Pose 2:     -2.1895034948932093     =   -2.1895
                -1.7199257055865687     =   -1.7199
                -1.876540486012594      =   -1.8765
                -2.911086384450094      =   -2.9111
                -1.4573000113116663     =   -1.4573
                -1.501343075429098      =   -1.5302         1.5013
    
    Pose 3:     -2.1901634375201624     =   -2.1902
                -1.6891844908343714     =   -1.6892
                -1.8755343596087855     =   -1.8755
                -2.9110501448260706     =   -2.9110
                -1.457264248524801      =   -1.4573
                -0.5894625822650355     =   -2.3562         0.5895
    
    Pose 4:     -2.189803425465719      =   -2.1898
                -1.688608948384402      =   -1.6886
                -1.8755343596087855     =   -1.8755
                -2.910990540181295      =   -2.9110
                -1.4574316183673304     =   -1.4574
                -2.59609824815859       =   -0.8785         2.5961

    Pose 5:     -2.3546555677997034     =   -2.3546
                -1.7499845663653772     =   -1.7450
                -1.8335226217852991     =   -1.8335
                -2.907560173665182      =   -2.9076
                -1.4556463400470179     =   -1.4556
                -2.742664400731222      =   -2.7427
    
    Pose 6:     -1.9104340712176722     =   -1.9104
                -1.8239052931415003     =   -1.8239
                -1.9944971243487757     =   -1.9945
                -2.9075000921832483     =   -2.9075
                -1.457623306904928      =   -1.4576
                -3.952686134968893      =   -3.9270         3.9527


    -1.5302 --> -2.3562 --> -0.8785 --> -3.9270
    NEW AND REAL POSES!!!!!
    Shoulder_pan_joint, shoulder_lift_joint, elbow_joint, Wrist_1_joint, Wrist_2_joint, wrist_3_joint
    Initial pose: 
        joints = [-1.95703870455 -1.24395019213 -2.39311916033 -0.233633343373 -4.47188872496 -1.77920324007]
        ee_link pose = [
        header: 
        seq: 0
        stamp: 
            secs: 1557994693
            nsecs: 239372014
        frame_id: "/world"
        pose: 
        position: 
            x: 0.207742935132
            y: 0.273160977518
            z: 0.322503768222
        orientation: 
            x: -0.651284098889
            y: -0.274518622238
            z: 0.631300409869
            w: 0.319262182348 ]
        ee_link RPY = [-1.569572180120941, 0.7036648331665561, 1.5022795454934894]
    Pose 1:
        joints = [-1.96948463122 -1.57098800341 -2.51613933245 0.479521155357 -4.43353170553 -1.75650769869]
        ee_link pose = [
        header: 
        seq: 0
        stamp: 
            secs: 1557994984
            nsecs: 621098041
        frame_id: "/world"
        pose: 
        position: 
            x: 0.207946852452
            y: 0.28601323153
            z: 0.245128806875
        orientation: 
            x: -0.620724888827
            y: -0.369267304856
            z: 0.575827141939
            w: 0.383099690109 ]
        ee_link RPY = [-1.618759768622739, 0.44656246331761146, 1.4997841778117509]
    Pose 2:
        joints = [-1.92150050799 -1.46588260332 -2.38986069361 0.526149868965 -3.93682009379 -0.926803414022]
        ee_link pose = [
        header: 
        seq: 0
        stamp: 
            secs: 1557995229
            nsecs: 451776981
        frame_id: "/world"
        pose: 
        position: 
            x: 0.160849234458
            y: 0.289688697094
            z: 0.336897409852
        orientation: 
            x: -0.259769064454
            y: -0.292029486063
            z: 0.789290606174
            w: 0.473560082174 ]
        ee_link RPY = [-0.7944049069333141, 0.1338314802596121, 2.0045348280958923]
    Pose 3:
        joints = [-1.9215725104 -1.46632606188 -2.36134797732 0.526149868965 -5.3005878369 -2.25063020388]
        ee_link pose = [
        header: 
        seq: 0
        stamp: 
            secs: 1557995307
            nsecs:  88376045
        frame_id: "/world"
        pose: 
        position: 
            x: 0.262853989132
            y: 0.267829457849
            z: 0.345874292027
        orientation: 
            x: -0.882053507504
            y: -0.258182050819
            z: 0.17802651893
            w: 0.351610860318 ]
        ee_link RPY = [-2.3398852153263827, 0.13289906888493172, 0.6258518101259305]
    Pose 4: NOT NECCESARRY!!!!
        joints = [-1.88440448443 -1.49218207995 -2.36245018641 0.521885752678 -4.27783018747 -0.722246948873]
        ee_link pose = [
        header: 
        seq: 0
        stamp: 
            secs: 1557995377
            nsecs: 710076093
        frame_id: "/world"
        pose: 
        position: 
            x: 0.180255368565
            y: 0.314314503497
            z: 0.335010854746
        orientation: 
            x: -0.268900595237
            y: -0.181456468958
            z: 0.727980271942
            w: 0.603995648509 ]
        ee_link RPY = [-0.6409820296747785, 0.1731653104072901, 1.6987694231362067]
    Pose 5:
        joints = [-2.42251998583 -2.20812732378 -2.08849555651 0.921115040779 -4.49157697359 -3.93944079081]
        ee_link pose = [
        header: 
        seq: 0
        stamp: 
            secs: 1557999672
            nsecs: 827539920
        frame_id: "/world"
        pose: 
        position: 
            x: 0.444792593824
            y: 0.268273817185
            z: 0.145345761126
        orientation: 
            x: 0.804802856886
            y: 0.458351877665
            z: 0.0704699310687
            w: 0.370459048484 ]
        ee_link RPY = [2.3959026509700427, 0.2281574481214959, 0.9458848055556739]
    Pose 6:
        joints = [-1.90473920504 -1.42789727846 -2.45266300837 0.150948762894 -4.56828874746 -1.59162122408]
        ee_link pose = [
        header: 
        seq: 0
        stamp: 
            secs: 1557995573
            nsecs: 327100038
        frame_id: "/world"
        pose: 
        position: 
            x: 0.206567480324
            y: 0.298457401516
            z: 0.27921197869
        orientation: 
            x: -0.632500151219
            y: -0.262257921445
            z: 0.603417778416
            w: 0.408719128544 ]
        ee_link RPY = [-1.496153813774184, 0.58112349392033, 1.4095527169318343]
    
    joints = [-1.94804603258 -1.37376910845 -2.45643669764 0.197997808456 -4.4299702088 -1.73460227648]
    ee_link pose = [
    header: 
    seq: 0
    stamp: 
        secs: 1558002261
        nsecs: 785916090
    frame_id: "/world"
    pose: 
    position: 
        x: 0.203197080103
        y: 0.278728370202
        z: 0.302917546322
    orientation: 
        x: -0.61642220537
        y: -0.356605361696
        z: 0.58849584922
        w: 0.382790956252 ]
    ee_link RPY = [-1.5868126290711906, 0.4695798159521042, 1.511365892698261]

'''

def getGripperStatus (data):
    # Get current data of gripper
    gripper_input.gACT = data.gACT
    gripper_input.gGTO = data.gGTO
    gripper_input.gSTA = data.gSTA
    gripper_input.gOBJ = data.gOBJ  
    gripper_input.gFLT = data.gFLT
    gripper_input.gPR = data.gPR
    gripper_input.gPO = data.gPO
    gripper_input.gCU = data.gCU

class Cupmovement ():
    def __init__ (self):
        self.startJointValues = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.prevJointValues = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.jointValues = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pass
    
    def moveArm(self): # Move the arm
        planned_path = moveitGroup.plan(self.jointValues) # Plan a path to joint positions
        moveitGroup.execute(planned_path, wait=True) # Move and wait till it has finished
        #desiredCase = change_state.next(presentCase) # Get new state
        #presentCase += desiredCase # The new state is the presentstate which it should execute
        #rospy.sleep(2) # Wait for 1 second
    
    def initialPose(self): # Get start values
        self.jointValues[0] = -1.95703870455
        self.jointValues[1] = -1.24395019213
        self.jointValues[2] = -2.39311916033
        self.jointValues[3] = -0.233633343373
        self.jointValues[4] = -4.47188872496
        self.jointValues[5] = -1.77920324007
        self.startJointValues = self.jointValues
        self.moveArm()
        self.prevJointValues = self.jointValues

    def pose1(self):
        #global jointValues, prevJointValues, startJointValues
        print "Pose 1: " + str(self.startJointValues)
        self.jointValues[0] = self.startJointValues[0] - 0.0125
        self.jointValues[1] = self.startJointValues[1] - 0.327
        self.jointValues[2] = self.startJointValues[2] - 0.123
        self.jointValues[3] = self.startJointValues[3] + 0.7131
        self.jointValues[4] = self.startJointValues[4] + 0.0384
        self.jointValues[5] = self.startJointValues[5] + 0.0227
        self.moveArm()
        self.prevJointValues = self.jointValues
        gripper.moveGripper(190)

    def pose2(self):
        print "Pose 2: " + str(self.startJointValues)
        self.jointValues[0] = self.prevJointValues[0] #+ 0.0309
        self.jointValues[1] = self.prevJointValues[1] + 0.1051
        self.jointValues[2] = self.prevJointValues[2] + 0.1262
        self.jointValues[3] = self.prevJointValues[3] + 0.0466
        self.jointValues[4] = self.prevJointValues[4] #+ 0.4967
        self.jointValues[5] = self.prevJointValues[5] + 0.8297
        self.moveArm()
        self.prevJointValues = self.jointValues

    def pose3_1(self):
        self.jointValues[0] = self.prevJointValues[0]
        self.jointValues[1] = self.prevJointValues[1]
        self.jointValues[2] = self.prevJointValues[2]
        self.jointValues[3] = self.prevJointValues[3]
        self.jointValues[4] = -4.555 #self.prevJointValues[4] -3.9368
        self.jointValues[5] = -0.9268 #self.prevJointValues[5]
        self.moveArm()
        self.prevJointValues = self.jointValues

    def pose3(self):
        print "Pose 3: " + str(self.startJointValues)
        self.jointValues[0] = self.prevJointValues[0]
        self.jointValues[1] = self.prevJointValues[1]
        self.jointValues[2] = self.prevJointValues[2]
        self.jointValues[3] = self.prevJointValues[3] 
        self.jointValues[4] = self.prevJointValues[4] - 0.445 #1.3638
        self.jointValues[5] = self.prevJointValues[5] - 1.3238
        self.moveArm()
        self.prevJointValues = self.jointValues

    def pose5(self): #CHANGES ACCORDING TO POSE 3
        print "Pose 5: " + str(self.startJointValues)
        self.jointValues[0] = self.prevJointValues[0] - 0.5009
        self.jointValues[1] = self.prevJointValues[1] - 0.7418
        self.jointValues[2] = self.prevJointValues[2] + 0.2728
        self.jointValues[3] = self.prevJointValues[3] + 0.395
        self.jointValues[4] = self.prevJointValues[4] + 0.5084
        self.jointValues[5] = self.prevJointValues[5] - 1.6888
        self.moveArm()
        self.prevJointValues = self.jointValues

    def pose6(self):
        self.jointValues[0] = self.prevJointValues[0] + 0.4745
        self.jointValues[1] = self.prevJointValues[1] + 0.8343
        self.jointValues[2] = self.prevJointValues[2] - 0.3679
        self.jointValues[3] = self.prevJointValues[3] - 0.7231
        self.jointValues[4] = self.prevJointValues[4] + 0.0616
        self.jointValues[5] = self.prevJointValues[5] + 2.2048
        self.moveArm()
        self.prevJointValues = self.jointValues

    def pose7(self):
        print "Pose 7: " + str(self.startJointValues)
        self.jointValues[0] = -1.95703870455 - 0.0125
        self.jointValues[1] = -1.24395019213 - 0.327
        self.jointValues[2] = -2.39311916033 - 0.123
        self.jointValues[3] = -0.233633343373 + 0.7131
        self.jointValues[4] = -4.47188872496 + 0.0384
        self.jointValues[5] = -1.77920324007 + 0.227
        self.moveArm()
        self.prevJointValues = self.jointValues
        gripper.moveGripper(0)

class gripperCommand():
    def __init__(self):
        # The gripper should be attached with your laptop through the usb
        # Make sure the gripper node is launched (rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0)
        # Initialize gripper
        print "initializing gripper"
        gripper_command.rACT = 1 # Activate gripper
        gripper_command.rGTO = 1 # Gripper is able to go to requested position
        gripper_command.rSP  = 200 # Speed of opening and closing gripper is 200 0 = minimum speed, 255 = maximum speed
        gripper_command.rFR  = 5 # Force of grabbing the object 0 = minimum force, 255 = maximum force
        gripper_publisher.publish(gripper_command) # Publish the registers
        rospy.sleep(0.2) # Wait for 0.2 seconds to make sure it is received
        gripper_command.rACT = 0 # Deactivate the gripper
        gripper_publisher.publish(gripper_command) # Publish the registers 
        rospy.sleep(0.2) # Wait for 0.2 seconds to make sure it is received
        gripper_command.rACT = 1 # Reactivate the gripper
        gripper_publisher.publish(gripper_command) # Publish the registers 
        rospy.sleep(0.2) # Wait for 0.2 seconds to make sure it is received
        self.moveGripper(0) # Open the gripper
        gripper_publisher.publish(gripper_command) # Publish the registers

        if gripper_input.gSTA != 0x03: # If gripper is not activated return 
            return
        rospy.loginfo("gripper activated")

    def moveGripper(self, distance):
        gripper_command.rPR = distance # Move the gripper to desired distance
        gripper_publisher.publish(gripper_command) # Publish the registers

if __name__ == "__main__":
    try:
        rospy.init_node("shakeAndThrowCup", anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv) # Initialize the moveit commander
        robotcommander = moveit_commander.RobotCommander() # Bound the robot commander to a variable
        rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, getGripperStatus) # Subscribe to the gripper registers to get the status
        gripper_publisher = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=20) # Publish gripper values
        gripper_command = outputMsg.Robotiq2FGripper_robot_output() # Bound output messages to a variable
        gripper_input = inputMsg.Robotiq2FGripper_robot_input() # Bound input messages to a variable

        group_name = "manipulator" # Group name of UR5 robotic arm is called manipulator
        moveitGroup = moveit_commander.MoveGroupCommander(group_name) # Move the group name 

        gripper = gripperCommand() # Bound class GripperCommand
        move = Cupmovement()

        while not rospy.is_shutdown():

            print "PresentCase = " + str(presentCase)
            if presentCase == 0:
                move.initialPose() # go to begin position
                presentCase = presentCase + 1
            elif presentCase == 1:
                move.pose1() # grab the cup
                if gripper_input.gOBJ == 2 or gripper_input.gPO > 180:
                    presentCase = presentCase + 1
                    rospy.sleep(0.01)
            elif presentCase == 2:
                move.pose2()
                presentCase = presentCase + 1
            elif presentCase == 3:
                for n in range(3):
                    move.pose3_1() # shake position 1
                    move.pose3() # shake position 2
                presentCase = presentCase + 2
            elif presentCase == 4:
                move.pose4() # Ready to throw dices
                presentCase = presentCase + 1
            elif presentCase == 5:
                move.pose5() # dices are thrown
                presentCase = presentCase + 1
            elif presentCase == 6:
                move.pose6() # Above cup place
                presentCase = presentCase + 1
            elif presentCase == 7:
                move.pose7() # Release cup on original place
                if gripper_input.gOBJ != 2 or gripper_input.gPO < 10:
                    presentCase = 0
                    rospy.sleep(0.01)
            
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupt detected, stopping the program")
        gripper_command.rACT(0) # Deactivate the gripper
        gripper_publisher.publish(gripper_command) # Publish the registers 
        rospy.sleep(0.2) # Wait for 0.2 seconds to make sure it is received