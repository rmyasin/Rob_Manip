#!/usr/bin/env python

# Rosbag recording script that allows starting and stopping recording from scriptline
# Is mostly just a hack to run the bash script for "rosbag record" and then to kill all nodes starting with "record_"
# Adapted from https://gist.github.com/marco-tranzatto/8be49b81b1ab371dcb5d4e350365398a
# Which in turn was inspired by responses at https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/

import rospy
import subprocess
import os
import signal
import ipdb
import std_msgs.msg as std_msgs
import sys

class RosbagRecord:

    def __init__(self,fileInput='',folderInput='',start=False):
        
        self.b_recording=False
        self.filename=fileInput
        self.foldername=folderInput
        

        # If the foldername is empty, use the working directory
        if not self.foldername:
            self.set_foldername(os.getcwd())

        # Make sure to close cleanly on shutdown
        rospy.on_shutdown(self.stop_recording)

        # Start recording
        if start:
           self.start_recording()

    def terminate_ros_node(self, s):
        # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)

    def set_filename(self,fileInput):
        #TODO: set checks on filename?
        self.filename=fileInput

    def set_foldername(self,folderInput):
        #TODO: set checks on foldername/make it if it doesn't exist
        self.foldername=folderInput

    def start_recording(self,method='all',b_param=False):
        commandList= \
        {   'all': 'rosbag record -q -a -x /set_continuous_palpation_trajectory --split --duration=4m  ',
            'robot': 'rosbag record -q --split --duration=4m -e \'(?!/stereo).*\' -x /set_continuous_palpation_trajectory',
            'vision': 'rosbag record --lz4 -q --split --duration=4m -e \'/stereo/right/image_rect|/stereo/right/camera_info|/stereo/left/image_rect|/stereo/left/camera_info\' -x /set_continuous_palpation_trajectory',
        }
        if (not self.b_recording) and os.path.exists(self.foldername):
            # Start recording
            command = commandList[method]
            if self.filename:
                command = command + ' -o ' + self.filename
            subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self.foldername,
                                      executable='/bin/bash')
            if b_param:
                paramDumpCommand = 'rosparam dump ' + self.filename +'Params.yaml'
                subprocess.Popen(paramDumpCommand, stdin=subprocess.PIPE, shell=True, cwd=self.foldername,
                                      executable='/bin/bash')
        self.b_recording=True

    def stop_recording(self):
        if self.b_recording:
            rospy.loginfo(rospy.get_name() + ' stop recording.')
            self.terminate_ros_node("/record_")
        self.b_recording=False

    def new_recording(self,fileInput='',folderInput='',method='all'):
        self.stop_recording()
        if fileInput:
            self.set_filename(fileInput)
        if folderInput:
            self.set_foldername(folderInput)
        self.start_recording(method)

#---------------------------------------------------
# Set up some basic functions with user interaction
# for testing class functionality
#---------------------------------------------------
user_options = ['Quit',
                'Filename', 
                'Foldername',
                'Start Recording',
                'Stop Recording',
                'Print Options',
                'Debug']   
def print_options():
    i = 0;
    print ''
    for str in user_options:
        print i, (' : ' + str)
        i = i+1
        pass
    pass

if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    argMethod=''
    fullName=''
    folderPath=''
    print str(myargv)
    if len(myargv)>1:
        argMethod=myargv[1]
    if len(myargv)>2:
        fullName=myargv[2]
    if len(myargv)>3:
        folderPath=myargv[3]
    print fullName

    # Start the base node for recording
    # if rospy.is_shutdown():
    rospy.init_node('rosbag_record')

    if argMethod == "continuous":
        filename = str(rospy.rostime.get_rostime()) + 'continous_palpation'
        if not folderPath:
            folderPath = '/home/arma/catkin_ws/data/continuous_palpation'
        rosbag_record = RosbagRecord(filename,folderPath,False)
        while len(rospy.get_param_names())<5:
            pass
        rosbag_record.start_recording('robot',b_param=True)

        while not rospy.is_shutdown():
            pass
        rosbag_record.stop_recording()
    elif argMethod =="path":
        filename = str(rospy.rostime.get_rostime()) + 'path_following'
        if fullName:
            filename=fullName
        if not folderPath:
            folderPath = '/home/arma/catkin_ws/data/path_following'
        rosbag_record = RosbagRecord(filename,folderPath,False)
        rosbag_record.start_recording('robot',b_param=False)
        while not rospy.is_shutdown():
            pass
        rosbag_record.stop_recording()
    elif argMethod == "micron":
        filename = "micron"
        if fullName:
            filename=fullName
        if not folderPath:
            folderPath = '/home/arma/catkin_ws/data/micron'
        rosbag_record = RosbagRecord(filename,folderPath,False)
        rosbag_record.start_recording('all',b_param=False)
        while not rospy.is_shutdown():
            pass
        rosbag_record.stop_recording()
    else:
        filename= 'testA'
        if not folderPath:
            folderPath = '/home/arma/catkin_ws/data'

        
        try:
            # Create a class instance to do the recording, add options to stop recording/change filenames
            rosbag_record = RosbagRecord(filename,folderPath,True)
            opt = '111'
            print_options()
            while int(opt) != 0:
                opt = raw_input('\nEnter option : ')
                if opt is '0':
                    quit()
                elif opt is '1':
                    fileInput = raw_input('\nEnter filename: ')
                    rosbag_record.set_filename(fileInput)
                elif opt is '2':
                    folderInput = raw_input('\nEnter foldername: ')
                    rosbag_record.set_foldername(folderInput)
                elif opt is '3':
                    rosbag_record.start_recording()
                elif opt is '4':
                    rosbag_record.stop_recording()
                elif opt is '5':
                    print_options()
                elif opt is '6':
                    ipdb.set_trace()
                pass
            pass
        except rospy.ROSInterruptException:
            pass
