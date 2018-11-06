import os
import math
import pandas as pd

import yaml
# from ruamel.yaml import yaml
# import ruamel.yaml as yaml
import rospy
import rospkg
import rosservice
import tf
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import visualization_msgs.msg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QErrorMessage
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem, QDoubleValidator

# Object map to save in file yaml or cfg
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
        """
        self.map_path_name = ''
        self.map_file_name = ''
        self.map_forRviz_file_name = ''
        self.destination_map_file_name = ''
        self.destination_map_config_file_name = ''
        self.cameramap = []
        self.current_wp = None

        """
        Connect stuff here
        """
        # self.model = QStandardItemModel(self._widget.arc_list)
        # self._widget.name_edit.setValidator(QDoubleValidator())
        # self._widget.path_edit.setValidator(QDoubleValidator())
        # self._widget.in_case_edit.setValidator(QDoubleValidator())
        # self._widget.y_edit.setValidator(QDoubleValidator())
        # self._widget.map_name_edit.setValidator()

        self._widget.file_name_button.released.connect(self.get_file)
        self._widget.desination_file_name_button.released.connect(self.get_folder)
        self._widget.desination_save_file_button.released.connect(self.save_file)
        self._widget.config_map_file_name_button.released.connect(self.get_file_yaml)

        # Buttons
        self._widget.config_map_save_file_button.released.connect(self.config_save_file)
        self._widget.map_to_rviz_send_file_button.released.connect(self.visualize_fiducials)
        self._widget.map_to_rviz_name_button.released.connect(self.get_file_map_to_rviz)

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
        ROS
        """
        self.marker_pub = rospy.Publisher('/fiducials_position', visualization_msgs.msg.MarkerArray, queue_size=10)
        # empty yet

    """
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
        # file_dlg.setFilter("Yaml files (*.yaml)")
        # print('Hi, you try to get a file')

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
        # QFileDialog::getExistingDirectory()
        file_dlg.setFileMode(QFileDialog.Directory)
        # getExistingDirectory(self, "Open a folder", "/home/", file_dlg.ShowDirsOnly)
        # # file_dlg.setFilter("Yaml files (*.yaml)")
        # # print('Hi, you try to get a file')
        #
        # fileName = QFileDialog::getOpenFileName(this, tr("Open Image"), "/home/catkin_ws_kyb", tr("Image Files (*.png *.jpg *.bmp)"));
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

                # self.read_waypoints()
    """
    """
    def visualize_fiducials(self, event=None):
        data = pd.read_csv(self.map_forRviz_file_name, header=None, sep=' ', skipinitialspace=True, low_memory=False, skiprows=3)
        fiducial_pos = data.values[:,1], data.values[:,14], data.values[:,15], data.values[:,16] #concatanate the vector

        # delete all fiducial
        marker_array = visualization_msgs.msg.MarkerArray()
        new_marker = visualization_msgs.msg.Marker()
        new_marker.header.stamp = rospy.Time.now()
        # Set frame id
        new_marker.header.frame_id = 'odom'
        new_marker.ns = 'fiducials_pos'
        new_marker.action = visualization_msgs.msg.Marker.DELETEALL;
        marker_array.markers.append(new_marker)
        self.marker_pub.publish(marker_array)

        print 'Prepare and send data'
        # print(fiducial_pos[0][158])
        # print type(fiducial_pos[0][1])
        # Send fiducial


        length = len(fiducial_pos[0]) - 1
        marker_array = visualization_msgs.msg.MarkerArray()
        for i in range(0, length):
            new_marker = visualization_msgs.msg.Marker()
            new_marker.header.stamp = rospy.Time.now()
            new_marker.header.frame_id = 'odom'
            new_marker.id = i #int(fiducial_pos[i][0])
            new_marker.ns = 'fiducials position'
            new_marker.type = visualization_msgs.msg.Marker.SPHERE;
            new_marker.action = visualization_msgs.msg.Marker.ADD;
            new_marker.pose.position.x = fiducial_pos[1][i]
            new_marker.pose.position.y = fiducial_pos[2][i]
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
        self.marker_pub.publish(marker_array)

        # Fiducial names
        marker_array = visualization_msgs.msg.MarkerArray()
        for i in range(0, length):
            new_marker = visualization_msgs.msg.Marker()
            new_marker.header.stamp = rospy.Time.now()
            new_marker.header.frame_id = 'odom'
            new_marker.id = i #int(fiducial_pos[i][0])
            new_marker.ns = 'fiducials_names'
            new_marker.type = visualization_msgs.msg.Marker.TEXT_VIEW_FACING;
            new_marker.action = visualization_msgs.msg.Marker.ADD;
            new_marker.text = str(fiducial_pos[0][i])
            new_marker.pose.position.x = fiducial_pos[1][i]
            new_marker.pose.position.y = fiducial_pos[2][i]
            new_marker.pose.position.z = .1
            new_marker.scale.z = 0.1
            new_marker.color.a = 1.0
            new_marker.color.r = 1.0
            new_marker.color.g = 1.0
            new_marker.color.b = 1.0
            marker_array.markers.append(new_marker)
        self.marker_pub.publish(marker_array)

    """
    """
