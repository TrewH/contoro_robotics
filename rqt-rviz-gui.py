import os
import rospy
from PyQt5.QtWidgets import QWidget, QPushButton
from python_qt_binding import loadUi
from rqt_gui_py.plugin import Plugin
from rviz import bindings as rviz
from unloading_robot_hardware_verification.srv import (
    trajectorySelect,
    trajectorySelectRequest,
    trajectorySelectResponse,
)


class GuiPlugin(Plugin):

    def __init__(self, context):
        super(GuiPlugin, self).__init__(context)
        # Give QObjects names
        self.setObjectName("GuiPlugin")
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "resource/rqt_mygui.ui"
        )
        # Extend widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        self._rviz_frame = rviz.VisualizationFrame()
        self._rviz_frame.setSplashPath("")
        self._rviz_frame.initialize()

        self._widget.layout().addWidget(self._rviz_frame)

        # Add widget to the user interface
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )

        context.add_widget(self._widget)

        self._manager = self._rviz_frame.getManager()

        self._trajectory_display = self._manager.createDisplay(
            "rviz/Path", "Trajectory Display", True
        )

        button_one = self._widget.findChild(QPushButton, "button_one")
        button_one.clicked.connect(
            lambda action: self.on_button_press(action, "traj_one")
        )

        button_two = self._widget.findChild(QPushButton, "button_two")
        button_two.clicked.connect(
            lambda action: self.on_button_press(action, "traj_two")
        )

        button_three = self._widget.findChild(QPushButton, "button_three")
        button_three.clicked.connect(
            lambda action: self.on_button_press(action, "traj_three")
        )

        button_four = self._widget.findChild(QPushButton, "button_four")
        button_four.clicked.connect(
            lambda action: self.on_button_press(action, "traj_four")
        )

    def on_button_press(self, action: bool, desired_trajectory: str) -> None:
        rospy.wait_for_service("trajectory_select_service")
        try:
            trajectory_service = rospy.ServiceProxy(
                "trajectory_select_service", trajectorySelect
            )
            request = trajectorySelectRequest(desiredTrajectory=desired_trajectory)
            response: trajectorySelectResponse = trajectory_service(request)
            rospy.loginfo(
                f"Service call success: {response.success}, message: {response.response}"
            )
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def cycle_trajectories(self) -> None:
        self.current_index = (self.current_index + 1) % len(self.trajectory_topics)
        next_topic = self.trajectory_topics[self.current_index]
        self._trajectory_display_subprop.setValue(next_topic)

    def shutdown_plugin(self) -> None:
        pass
