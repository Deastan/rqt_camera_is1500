# By Jonathan Burkhard, Kyburz 2018
# Based on Joao's GUI
import os
from math import radians, pow
import pandas as pd
import numpy as np
import yaml
import subprocess

import rospy
import rospkg
import rosservice
import tf
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import visualization_msgs.msg

import tf.transformations
from geometry_msgs.msg import Quaternion, Point, Pose, Twist, Vector3
from nav_msgs.msg import Odometry

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QErrorMessage
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem, QDoubleValidator

# Object "map" to save in file yaml or cfg
class CameraMap:
    def __init__(self, name=''):
        self.name = name
        self.path = '/home/'

    def from_yaml(self, elem):
        self.name = elem['name']
        self.path = elem['path']

    def __repr__(self):
        return self.name

class Camera_is1500_Widget(Plugin):
    def __init__(self, context):
        super(Camera_is1500_Widget, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Camera setting')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resources" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_camera_is1500'), 'resources', 'Camera_is1500_Widget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Camera_is1500_WidgetUI')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        """
        MAP path for is1500 package
        """
        self.map_path_name = ''
        self.map_file_name = ''
        self.destination_map_file_name = ''
        self.destination_map_config_file_name = ''
        self.cameramap = []
        self.current_wp = None
        """
        RVIZ-APPs
        """
        self.map_forRviz_file_name = self._widget.map_to_rviz_path_edit.text()
        self.utm_x_value = float(self._widget.utm_x_edit.text())
        self.utm_y_value = float(self._widget.utm_y_edit.text())
        self.utm_phi_value = float(self._widget.utm_phi_edit.text())
        self.joao_origin_utm_x_value = float(self._widget.joao_origin_utm_x_edit.text())
        self.joao_origin_utm_y_value = float(self._widget.joao_origin_utm_y_edit.text())
        self.fiducials_map = [] # contain the position of the fiducial in camera frame
        self.indoor = self._widget.indoor_checkBox_edit.isChecked()
        self.init_transf = False # True if map already drew
        self.origin_cam_x = 0.0
        self.origin_cam_y = 0.0
        """
        Connect stuff here
        """
        # self.model = QStandardItemModel(self._widget.arc_list)
        # self._widget.name_edit.setValidator(QDoubleValidator())
        # self._widget.path_edit.setValidator(QDoubleValidator())
        # self._widget.in_case_edit.setValidator(QDoubleValidator())
        # self._widget.y_edit.setValidator(QDoubleValidator())
        # self._widget.map_name_edit.setValidator()

        self._widget.launch_camera_start_button.released.connect(self.start_camera_is1500_launchFile)
        self._widget.color_launch_camera_label.setStyleSheet("background-color:#ff0000;")
        self._widget.launch_supervisor_start_button.released.connect(self.start_supervisor_launchFile)
        self._widget.launch_supervisor_color_label.setStyleSheet("background-color:#ff0000;")
        self._widget.rviz_start_button.released.connect(self.start_rviz)
        self._widget.color_rviz_label.setStyleSheet("background-color:#ff0000;")

        self._widget.test_button.released.connect(self.test_button_function)

        """
        Set map
        """
        self._widget.send_map_param_button.released.connect(self.set_map_rosparam)
        self._widget.map_param_edit.setValidator(QDoubleValidator())

        """
        Add/modify map
        """
        self._widget.file_name_button.released.connect(self.get_file)
        self._widget.desination_file_name_button.released.connect(self.get_folder)
        self._widget.desination_save_file_button.released.connect(self.save_file)
        self._widget.config_map_file_name_button.released.connect(self.get_file_yaml)
        self._widget.config_map_save_file_button.released.connect(self.config_save_file)
        self.model = QStandardItemModel(self._widget.map_from_mapYaml_list)
        self.model.itemChanged.connect(self.on_change_mapList)

        # Screen explaination msg
        # self._widget.name_edit.setToolTip("Set the name of the folder which will contain the map ")
        # self._widget.path_edit.setToolTip("Path of th map")
        # self._widget.file_name_label.setToolTipe("Map file which be added to the map folder of the robot")
        # Doesn't work
        # self._widget.name_edit.description = "This is label name_edit"
        # self._widget.path_edit.description = "This is label path_edit"
        # self._widget.file_name_button.description = "This is the OK button"


        #
        # for widget in (self._widget.name_edit, self._widget.path_edit,
        #     self._widget.file_name_button):
        #     widget.bind("<Enter>", self.on_enter)
        #     widget.bind("<Leave>", self.on_leave)
        self._widget.map_name_edit.textChanged.connect(self.map_name_change)

        """
        Rviz part
        """
        self._widget.utm_x_edit.setValidator(QDoubleValidator())
        self._widget.utm_y_edit.setValidator(QDoubleValidator())
        self._widget.utm_phi_edit.setValidator(QDoubleValidator())
        self._widget.utm_x_edit.textChanged.connect(self.utm_x_change)
        self._widget.utm_y_edit.textChanged.connect(self.utm_y_change)
        self._widget.utm_phi_edit.textChanged.connect(self.utm_phi_change)
        self._widget.joao_origin_utm_x_edit.setValidator(QDoubleValidator())
        self._widget.joao_origin_utm_y_edit.setValidator(QDoubleValidator())
        self._widget.joao_origin_utm_x_edit.textChanged.connect(self.joao_origin_utm_x_change)
        self._widget.joao_origin_utm_y_edit.textChanged.connect(self.joao_origin_utm_y_change)
        self._widget.indoor_checkBox_edit.stateChanged.connect(self.indoor_isCheck)
        self._widget.reset_init_map_file_button.released.connect(self.reset_init_change)

        # Buttons
        self._widget.map_to_rviz_send_file_button.released.connect(self.visualize_fiducials)

        self._widget.map_to_rviz_name_button.released.connect(self.get_file_map_to_rviz)
        """
        ROS
        """
        self.marker_pub = rospy.Publisher('/fiducials_position', visualization_msgs.msg.MarkerArray, queue_size=10) # publish fiducials for a chosen map
        self.currentMap_marker_pub = rospy.Publisher('/fiducials_position_current', visualization_msgs.msg.MarkerArray, queue_size=10)#publish currend used fiducials
        # self.camera_pos_sub = rospy.Subscriber("/base_link_odom_camera_is1500", Odometry, self.publish_transform_pos)
        self.camera_pos_sub = rospy.Subscriber("/position_camera_is1500", Odometry, self.publish_transform_pos)
        self.transform_cameraPos_pub = rospy.Publisher('/transform_cameraPos_pub', Odometry, queue_size=1)
        # rospy.spin()
        # empty yet

    """
    Start nodes
    """
    def start_camera_is1500_launchFile(self):
        # os.spawnl(os.P_NOWAIT, 'sudo shutdown && 123456')
        subprocess.call('/home/jonathan/catkin_ws_kyb/src/rqt_camera_is1500/script/camera_is1500.bash', shell=True)
        # os.spawnl(os.P_NOWAIT, 'cd && cd /home/jonathan/catkin_ws_kyb/ && source devel/setup.bash && cd src/camera_is1500/launch/ && roslaunch camera_is1500 cameraTry.launch')
        # print 'Node: Camera_is1500 launched'
        self._widget.color_launch_camera_label.setStyleSheet("background-color:#228B22;")

    def start_supervisor_launchFile(self):
        cmd = "gnome-terminal -x sh -c 'cd && cd /home/jonathan/catkin_ws_kyb/ && source devel/setup.bash && roslaunch supervisor supervisor.launch"
        subprocess.call(cmd, shell=True)
        self._widget.launch_supervisor_color_label.setStyleSheet("background-color:#228B22;")

    def start_rviz(self):
        self._widget.color_rviz_label.setStyleSheet("background-color:#ff0000;")
        subprocess.call('/home/jonathan/catkin_ws_kyb/src/rqt_camera_is1500/script/rviz.bash', shell=True)
        # print 'RViz launched'
        self._widget.color_rviz_label.setStyleSheet("background-color:#228B22;")
    def test_button_function(self):
        print('Checkbox: ', self.indoor)

    """
    Node param settings
    """
    def set_map_rosparam(self):
        cmd = "rosparam set /camera_is1500_node/mapNumber %d" % (float(self._widget.map_param_edit.text()))
        subprocess.call(cmd, shell=True)
    """
    Map selection function
    """
    def map_name_change(self):
        self.map_file_name = self._widget.map_name_edit.text()

    """
    """
    def read_map(self):
        self.cameramap = []
        with open(self.destination_map_config_file_name, 'r') as stream:
            map_list = yaml.load(stream)
            for map in map_list:
                new_map = CameraMap()
                new_map.from_yaml(map)
                self.cameramap.append(new_map)

        # self.populate_combobox()
        # rospy.loginfo('Read: %s'%self.cameramap)
        # print('Read: %s'%self.cameramap)
        print('Load data')
        for i in range(0, len(self.cameramap)):
            print(self.cameramap[i].name)
            print(self.cameramap[i].path)
        # self.wait_timer = rospy.Timer(rospy.Duration(2.), self.visualize_waypoints, oneshot=True)

    """
    """
    def config_save_file(self):
        map_list = []
        for map in self.cameramap:
            new_map = {}#CameraMap()#{}
            new_map['name'] = map.name
            new_map['path'] = map.path
            map_list.append(new_map)

        new_map_to_add = {}
        new_map_to_add['name'] = self.map_file_name
        new_map_to_add['path'] = self.destination_map_config_file_name
        map_list.append(new_map_to_add)

        with open(self.destination_map_config_file_name, 'w') as stream:
            stream.write(yaml.safe_dump(map_list, encoding='utf-8', allow_unicode=False, default_flow_style=False))
        print 'Saved elements in file' + self.destination_map_config_file_name
        print yaml.safe_dump(map_list, encoding='utf-8', allow_unicode=False, default_flow_style=False)

    """
    """
    def save_file(self):
        print('Hi, you are triing to save something with the name of : ',map_name_edit)
    """
    """
    def get_file(self):
        file_dlg = QFileDialog()
        file_dlg.setFileMode(QFileDialog.AnyFile)

        if file_dlg.exec_():
            map_file_name = file_dlg.selectedFiles()

            rospy.loginfo('Selected files: %s'%map_file_name)
            if len(map_file_name) > 0:
                self.map_path_name = map_file_name[0]
                self._widget.file_path_edit.setText(self.map_path_name)

                # self.read_waypoints()
    """
    """
    def get_folder(self):
        file_dlg = QFileDialog()
        file_dlg.setFileMode(QFileDialog.Directory)

        if file_dlg.exec_():
            destination_map_file_name = file_dlg.selectedFiles()

            rospy.loginfo('Selected files: %s'%destination_map_file_name)
            if len(destination_map_file_name) > 0:
                self.destination_map_file_name = destination_map_file_name[0]
                self._widget.desination_file_name_edit.setText(self.destination_map_file_name)

    """
    """
    def get_file_yaml(self):
        file_dlg = QFileDialog()
        file_dlg.setFileMode(QFileDialog.AnyFile)
        # file_dlg.setFilter("Yaml files (*.yaml)")
        # print('Hi, you try to get a file')
        # print(self.map_file_name)
        if file_dlg.exec_():
            destination_map_config_file_name = file_dlg.selectedFiles()

            rospy.loginfo('Selected files: %s'%destination_map_config_file_name)
            if len(destination_map_config_file_name) > 0:
                self.destination_map_config_file_name = destination_map_config_file_name[0]
                self._widget.config_map_file_name_edit.setText(self.destination_map_config_file_name)

                self.read_map()
    """
    """
    # TODO : ........
    def on_change_mapList(self):
        print(hahaa)

    """
    RViz group
    """
    def utm_x_change(self):
        self.utm_x_value = float(self._widget.utm_x_edit.text())
        # print('utm_x: ', self.utm_x_value, ', type: ', type(self.utm_x_value))
    """
    """
    def utm_y_change(self):
        self.utm_y_value = float(self._widget.utm_y_edit.text())
        # print('utm_y: ', self.utm_y_value, ', type: ', type(self.utm_y_value))
    """
    """
    def utm_phi_change(self):
        self.utm_phi_value = float(self._widget.utm_phi_edit.text())
        # print('utm_phi: ', self.utm_phi_value, ', type: ', type(self.utm_phi_value))
    """
    """
    def joao_origin_utm_x_change(self):
        self.joao_origin_utm_x_value = float(self._widget.joao_origin_utm_x_edit.text())
    """
    """
    def joao_origin_utm_y_change(self):
        self.joao_origin_utm_y_value = float(self._widget.joao_origin_utm_y_edit.text())
    """
    """
    def get_file_map_to_rviz(self):
        file_dlg = QFileDialog()
        file_dlg.setFileMode(QFileDialog.AnyFile)
        # file_dlg.setFilter("Yaml files (*.yaml)")
        # print('Hi, you try to get a file')

        if file_dlg.exec_():
            map_file_name = file_dlg.selectedFiles()

            rospy.loginfo('Selected files to extract position of the fiducials: %s'%map_file_name)
            if len(map_file_name) > 0:
                self.map_forRviz_file_name = map_file_name[0]
                self._widget.map_to_rviz_path_edit.setText(self.map_forRviz_file_name)
    """
    """
    # init the map changement
    def reset_init_change(self):
        self.init_transf = False
        # delete all fiducial
        marker_array = visualization_msgs.msg.MarkerArray()
        new_marker = visualization_msgs.msg.Marker()
        new_marker.header.stamp = rospy.Time.now()
        # Set frame id
        new_marker.header.frame_id = 'odom'
        new_marker.ns = 'fiducials_pos'
        new_marker.action = visualization_msgs.msg.Marker.DELETEALL;
        marker_array.markers.append(new_marker)
        self.currentMap_marker_pub.publish(marker_array)
    """
    """
    # Screen in RViz a selected map in chosen position when button is preshed
    def visualize_fiducials(self, event=None):
        self.generalizedDrawingMap(self.map_forRviz_file_name,
            radians(self.utm_phi_value), [float(self.utm_x_value), float(self.utm_y_value)],
            [self.joao_origin_utm_x_value, self.joao_origin_utm_y_value], self.marker_pub)

    """
    """
    # Screen in RViz a map
    # Input:    data_path: path of the map from camera is 1500
    #           angleMap: compare to the east for x-axis in RAD
    #           originMapInUTM: Origin of the camera (usually #100) in UTM coordinates
    #           originGeneralUTM: Joao's origin
    # Output: screen map on RViz
    def generalizedDrawingMap(self, data_path, angleMap, originMapInUTM, originGeneralUTM, publisher):
                data = pd.read_csv(data_path, header=None, sep=' ', skipinitialspace=True, low_memory=False, skiprows=3)
                fiducial_pos = data.values[:,1], data.values[:,14], -data.values[:,15], data.values[:,16] #concatanate the vector
                self.fiducials_map = fiducial_pos
                # delete all fiducial
                marker_array = visualization_msgs.msg.MarkerArray()
                new_marker = visualization_msgs.msg.Marker()
                new_marker.header.stamp = rospy.Time.now()
                # Set frame id
                new_marker.header.frame_id = 'odom'
                new_marker.ns = 'fiducials_pos'
                new_marker.action = visualization_msgs.msg.Marker.DELETEALL;
                marker_array.markers.append(new_marker)
                publisher.publish(marker_array)

                joao_x = originGeneralUTM[0]#self.joao_origin_utm_x_value#468655
                joao_y = originGeneralUTM[1]#self.joao_origin_utm_y_value#5264080
                gps_origin_map_x = originMapInUTM[0]#float(self.utm_x_value)#468598.24
                gps_origin_map_y = originMapInUTM[1]#float(self.utm_y_value)#5264012.01

                # Calculate the size of the fiducials matrix
                length = len(fiducial_pos[0]) - 1
                # Fiducial points to RVIZ
                marker_array = visualization_msgs.msg.MarkerArray()
                for i in range(0, length,3):
                    new_marker = visualization_msgs.msg.Marker()
                    new_marker.header.stamp = rospy.Time.now()
                    new_marker.header.frame_id = 'odom'
                    new_marker.id = i #int(fiducial_pos[i][0])
                    new_marker.ns = 'fiducials position'
                    new_marker.type = visualization_msgs.msg.Marker.SPHERE;
                    new_marker.action = visualization_msgs.msg.Marker.ADD;
                    x = fiducial_pos[1][i]
                    y = fiducial_pos[2][i]
                    new_marker.pose.position.x = x * np.cos(angleMap) - y * np.sin(angleMap) + gps_origin_map_x - joao_x
                    new_marker.pose.position.y = x * np.sin(angleMap) + y * np.cos(angleMap) + gps_origin_map_y - joao_y
                    print('( ', new_marker.pose.position.x, ', ', new_marker.pose.position.y, ') ')
                    new_marker.pose.position.z = -fiducial_pos[3][i]
                    quat = tf.transformations.quaternion_from_euler(0, 0, 0)# TODO set the angle
                    new_marker.pose.orientation.x = quat[0]
                    new_marker.pose.orientation.y = quat[1]
                    new_marker.pose.orientation.z = quat[2]
                    new_marker.pose.orientation.w = quat[3]
                    new_marker.scale.x = 0.05
                    new_marker.scale.y = 0.05
                    new_marker.scale.z = 0.05
                    new_marker.color.a = 1.0
                    new_marker.color.r = 1.0
                    new_marker.color.g = 0.0
                    new_marker.color.b = 0.0
                    marker_array.markers.append(new_marker)
                publisher.publish(marker_array)

                # Fiducial names
                marker_array = visualization_msgs.msg.MarkerArray()
                for i in range(0, length, 3):
                    new_marker = visualization_msgs.msg.Marker()
                    new_marker.header.stamp = rospy.Time.now()
                    new_marker.header.frame_id = 'odom'
                    new_marker.id = i #int(fiducial_pos[i][0])
                    new_marker.ns = 'fiducials_names'
                    new_marker.type = visualization_msgs.msg.Marker.TEXT_VIEW_FACING;
                    new_marker.action = visualization_msgs.msg.Marker.ADD;
                    new_marker.text = str(fiducial_pos[0][i])
                    x = fiducial_pos[1][i]
                    y = fiducial_pos[2][i]
                    new_marker.pose.position.x = x * np.cos(angleMap) - y * np.sin(angleMap) + gps_origin_map_x - joao_x
                    new_marker.pose.position.y = x * np.sin(angleMap) + y * np.cos(angleMap) + gps_origin_map_y - joao_y
                    new_marker.pose.position.z = -fiducial_pos[3][i] + 0.1
                    new_marker.scale.z = 0.1
                    new_marker.color.a = 1.0
                    new_marker.color.r = 1.0
                    new_marker.color.g = 1.0
                    new_marker.color.b = 1.0
                    marker_array.markers.append(new_marker)
                publisher.publish(marker_array)

    """
    """

    def transformFromLastGPS(self, positionYaw, lastUTM):
        # TODO: Check where is these angles
        angle_btwXcam_East = radians(self.utm_phi_value)#45.0) # 0 = align with east for x and north = y
        last_gps_yaw = radians(0.0) #in radians
        wheel_odom_distance = 0.0  # meter

        # if the map is already drew... just send the transfrom position
        if(self.init_transf == False):
            # Use East as x-axis
            direction_vecteur = [float(wheel_odom_distance * np.cos(last_gps_yaw)), float(wheel_odom_distance * np.sin(last_gps_yaw))]
            # Robot position in UTM inside the map
            first_robot_x = lastUTM[0] + direction_vecteur[0]
            first_robot_y = lastUTM[1] + direction_vecteur[1]
            # Rotation from camera frame to UTM
            rotated_x = positionYaw[0]  * np.cos(angle_btwXcam_East) - (-1) * positionYaw[1] * np.sin(angle_btwXcam_East)
            rotated_y = positionYaw[0]  * np.sin(angle_btwXcam_East) + (-1) * positionYaw[1] * np.cos(angle_btwXcam_East)
            self.origin_cam_x = first_robot_x - rotated_x
            self.origin_cam_y = first_robot_y - rotated_y
            # Draw the selected map
            self.generalizedDrawingMap(self.map_forRviz_file_name,
                angle_btwXcam_East, [self.origin_cam_x, self.origin_cam_y],
                [self.joao_origin_utm_x_value, self.joao_origin_utm_y_value],
                self.currentMap_marker_pub)
            self.init_transf = True
        # print(origin_cam_x, ', ', origin_cam_x)
        x = positionYaw[0]
        y = positionYaw[1]
        yaw = positionYaw[2]
        # Rotation from camera frame to UTM
        # Translation
        x_prim = x * np.cos(angle_btwXcam_East) -(-1) * y * np.sin(angle_btwXcam_East) + self.origin_cam_x - self.joao_origin_utm_x_value
        # ATTENTION: ADD a minus here on y position to be on the map
        y_prim = (x * np.sin(angle_btwXcam_East) +(-1) * y * np.cos(angle_btwXcam_East)) + self.origin_cam_y - self.joao_origin_utm_y_value
        yaw_prim = -yaw + angle_btwXcam_East # TRUE !!!
        return x_prim, y_prim, yaw_prim
    """
    """
    def indoor_isCheck(self):
        self.indoor = self._widget.indoor_checkBox_edit.isChecked()
    """
    """
    def publish_transform_pos(self, msg):

        if self.indoor:
            # Angle in rad
            (roll, pitch, yaw_before) = tf.transformations.euler_from_quaternion(
                [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

            # Transform the camera data for Rviz
            # 468655
            # 5264080
            # 468598.24
            # 5264012.01
            x, y, yaw = self.transformFromLastGPS([msg.pose.pose.position.x,
                msg.pose.pose.position.y, yaw_before], [468655.0+0.0, 5264080.0-0.0])

            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0
            # Don't forget the "Quaternion()" and *
            odom.pose.pose.orientation = Quaternion(
                *tf.transformations.quaternion_from_euler(0, 0, yaw))
            # publisher
            self.transform_cameraPos_pub.publish(odom)

        # else:
        #     print('Outdoor')
