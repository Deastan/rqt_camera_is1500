import os
import math


import yaml
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
        Connect stuff here
        """
        # self.model = QStandardItemModel(self._widget.arc_list)
        self._widget.x_edit.setValidator(QDoubleValidator())
        self._widget.y_edit.setValidator(QDoubleValidator())
        self._widget.theta_edit.setValidator(QDoubleValidator())

        self._widget.file_name_button.released.connect(self.get_file)
        self._widget.save_file_button.released.connect(self.save_file)

        """
        ROS
        """


    """
    """
    def save_file(self):
        print('Hi, you try to save something')

    """
    """
    def get_file(self):
        file_dlg = QFileDialog()
        file_dlg.setFileMode(QFileDialog.AnyFile)
        # file_dlg.setFilter("Yaml files (*.yaml)")
        print('Hi, you try to get a file')
    """
    """
