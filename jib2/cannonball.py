# Do ball physics that we have already (drag and gravity)
# Part that we can move around the vector
# Launch button and velocity slider (point_publisher.py)

"""point_publisher.py

   Publish a point: x/y/z position, as set by three GUI sliders.

   This generates a point message, as well as a visualization marker
   array, which rviz can render.

   Node:      /point
   Publish:   /point                        geometry_msgs.msg.PointStamped
              /visualization_marker_array   visualization_msgs.msg.MarkerArray

"""

import rclpy
import numpy as np
import signal
import sys
import threading

from math                       import pi, sin, cos, acos, atan2, sqrt, fmod, exp
from hw5code.TransformHelpers   import *

from PyQt5.QtCore               import (Qt, QTimer)
from PyQt5.QtWidgets            import (QApplication, QFrame, QLineEdit,
                                        QWidget, QLabel, QHBoxLayout,
                                        QVBoxLayout, QSlider, QCheckBox)

from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from rclpy.time                 import Duration
from tf2_ros                    import TransformBroadcaster
from std_msgs.msg               import Bool, Float64
from geometry_msgs.msg          import Point, PointStamped, Vector3, Quaternion
from geometry_msgs.msg          import Pose, PoseStamped
from geometry_msgs.msg          import Transform, TransformStamped
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker, MarkerArray


#
#  Basic GUI Elements
#
class Slider(QWidget):
    def __init__(self, name, val, minval, maxval, callback):
        # Initialize the base class
        super().__init__()

        # Store the values needed
        self.min      = minval
        self.max      = maxval
        self.offset   = (maxval + minval) / 2.0
        self.slope    = (maxval - minval) / 2.0
        self.callback = callback

        # Top-Left: Name
        label = QLabel(name)
        label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        label.setMinimumWidth(40)

        # Top-Right: Number
        # self.number = QLabel("%6.3f" % self.value)
        # self.number.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        # self.number.setMinimumWidth(100)
        # self.number.setStyleSheet("border: 1px solid black;")
        self.number = QLineEdit()
        self.number.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.number.setMinimumWidth(100)
        self.number.setStyleSheet("border: 1px solid black;")
        self.number.editingFinished.connect(self.numberCB)

        # Bottom: Slider
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(-100, 100)
        self.slider.setFocusPolicy(Qt.NoFocus)
        self.slider.setPageStep(5)
        self.slider.valueChanged.connect(self.sliderCB)

        # Create the Layout
        hbox = QHBoxLayout()
        hbox.addWidget(label)
        hbox.addSpacing(10)
        hbox.addWidget(self.number)

        vbox = QVBoxLayout()
        vbox.addLayout(hbox)
        vbox.addWidget(self.slider)

        self.setLayout(vbox)

        # Set the initial value.
        self.set(val)

    def set(self, value):
        value = min(max(value, self.min), self.max)
        self.value   = value
        self.percent = int(100.0*(value-self.offset)/self.slope)
        self.number.setText("%6.3f" % self.value)
        self.slider.setValue(self.percent)

    def get(self):
        return self.value

    def numberCB(self):
        try:
            self.set(float(self.number.text()))
            self.callback(self.value)
        except:
            pass

    def sliderCB(self, percent):
        if not percent == self.percent:
            self.set(self.offset + self.slope * 0.01 * float(percent))
            self.callback(self.value)

class CheckBox(QWidget):
    def __init__(self, name, val, callback):
        # Initialize the base class
        super().__init__()

        # Store the values needed
        self.callback = callback

        # Checkbox
        self.checkbox = QCheckBox(name)
        self.checkbox.stateChanged.connect(self.CB)

        # Create the Layout
        vbox = QVBoxLayout()
        vbox.addWidget(self.checkbox)
        self.setLayout(vbox)

        # Set the initial value.
        self.set(val)

    def set(self, value):
        self.value = value
        self.checkbox.setChecked(value)

    def get(self):
        return self.value

    def CB(self):
        self.value = self.checkbox.isChecked()
        self.callback(self.value)
        
class Line(QWidget):
    def __init__(self):
        # Initialize the base class
        super().__init__()

        # Horizontal Line
        line = QFrame()
        line.setFrameShape(QFrame.HLine)

        # Create the Layout
        vbox = QVBoxLayout()
        vbox.addWidget(line)
        self.setLayout(vbox)


#
#  Composite GUI Elements
#
"""
class GUI_check(QWidget):
    def __init__(self, callback):
        # Initialize the base class
        super().__init__()

        # Initialize the state and save the callback function.
        self.value    = False
        self.callback = callback

        # Create the checkbox
        self.c = CheckBox('Check', self.value, self.CB)

        # Create the layout
        vbox = QVBoxLayout()
        vbox.addWidget(self.c)
        self.setLayout(vbox)

        # Set up the window.
        # self.setWindowTitle('Check')
        self.show()

    def kill(self, signum, frame):
        self.close()

    def set(self, value):
        self.value = value
        self.c.set(value)

    def get(self):
        return self.value

    def CB(self, val):
        self.value = val
        self.callback(self.value)

class GUI_data(QWidget):
    def __init__(self, callback):
        # Initialize the base class
        super().__init__()

        # Initialize the state and save the callback function.
        self.value    = 0.0
        self.callback = callback

        # Create the XYZ sliders
        self.s = Slider('Data', self.value, -1.0, 1.0, self.CB)

        # Create the layout
        vbox = QVBoxLayout()
        vbox.addWidget(self.s)
        self.setLayout(vbox)

        # Set up the window.
        # self.setWindowTitle('Data')
        self.show()

    def kill(self, signum, frame):
        self.close()

    def set(self, value):
        self.value = value
        self.s.set(value)

    def get(self):
        return self.value

    def CB(self, val):
        self.value = val
        self.callback(self.value)"""

class GUI_xyz(QWidget):
    def __init__(self, callback):
        # Initialize the base class
        super().__init__()

        # Select the initial values and ranges.
        value = [ 0.0,  0.0,  1.0]
        lower = [-1.0, -1.0,  0.0]
        upper = [ 1.0,  1.0,  2.0]
        name  = [ 'X',  'Y',  'Z']

        # Initialize the state and save the callback function.
        self.value    = value
        self.callback = callback

        # Create the XYZ sliders
        self.s = [Slider(name[i], value[i], lower[i], upper[i],
                         lambda x, ch=i: self.CB(ch,x)) for i in range(3)]

        # Create the layout
        vbox = QVBoxLayout()
        vbox.addWidget(self.s[0])
        vbox.addWidget(self.s[1])
        vbox.addWidget(self.s[2])
        self.setLayout(vbox)

        # Set up the window.
        # self.setWindowTitle('XYZ')
        self.show()

    def kill(self, signum, frame):
        self.close()

    def set(self, value):
        self.value = value
        for i in range(3):
            self.s[i].set(value[i])

    def get(self):
        return self.value

    def CB(self, ch, val):
        self.value[ch] = val
        self.callback(self.value)

"""class GUI_xyzrpy(QWidget):
    def __init__(self, callback):
        # Initialize the base class
        super().__init__()

        # Select the initial values and ranges.
        value = [ 0.0,  0.0,  1.0,    0.0,     0.0,   0.0]
        lower = [-1.0, -1.0,  0.0,    -pi,     -pi,   -pi]
        upper = [ 1.0,  1.0,  2.0,     pi,      pi,    pi]
        name  = [ 'X',  'Y',  'Z', 'Roll', 'Pitch', 'Yaw']

        # Initialize the state and save the callback function.
        self.value    = value
        self.callback = callback

        # Create the XYZ sliders
        self.s = [Slider(name[i], value[i], lower[i], upper[i],
                         lambda x, ch=i: self.CB(ch,x)) for i in range(6)]

        # Create the layout
        vbox = QVBoxLayout()
        vbox.addWidget(self.s[0])
        vbox.addWidget(self.s[1])
        vbox.addWidget(self.s[2])
        vbox.addWidget(Line())
        vbox.addWidget(self.s[3])
        vbox.addWidget(self.s[4])
        vbox.addWidget(self.s[5])
        self.setLayout(vbox)

        # Set up the window.
        # self.setWindowTitle('XYZ, RPY')
        self.show()

    def kill(self, signum, frame):
        self.close()

    def set(self, value):
        self.value = value
        for i in range(6):
            self.s[i].set(value[i])

    def get(self):
        return self.value

    def CB(self, ch, val):
        self.value[ch] = val
        self.callback(self.value)"""


class GUI_speed(QWidget):
    def __init__(self, callback):
        # Initialize the base class
        super().__init__()

        # Select the initial values and ranges.
        speed_0 = 0.0
        lower = 0.0
        upper = 47.0
        name  = 'Speed (m/s)'

        # Initialize the state and save the callback function.
        self.speed    = speed_0
        self.callback = callback

        # Create the XYZ sliders
        self.s = Slider(name, self.speed, lower, upper, self.CB)

        # Create the layout
        vbox = QVBoxLayout()
        vbox.addWidget(self.s)
        self.setLayout(vbox)

        # Set up the window.
        # self.setWindowTitle('XYZ')
        self.show()

    def kill(self, signum, frame):
        self.close()

    def set(self, value):
        self.value = value
        self.s.set(value)

    def get(self):
        return self.speed

    def CB(self, val):
        self.value = val
        self.callback(self.value)

#
#   GUI Node Class
#
class GUINode(Node):
    # Initialization.
    def __init__(self, name, rate):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Prepare the publishers.
        quality = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.pub_mark  = self.create_publisher(
            MarkerArray, '/visualization_marker_array', quality)
        """"
        self.pub_point = self.create_publisher(
            PointStamped, '/point', quality)
        """
        self.pub_speed = self.create_publisher(
            Float64, '/speed', quality)

        # Save the publisher rate and status.
        self.publishrate = rate
        self.publishing  = False

        """
        # Create the point.
        self.p = Point()

        # Create the point message.
        self.pointmsg = PointStamped()
        self.pointmsg.header.frame_id = "world"
        self.pointmsg.header.stamp    = self.get_clock().now().to_msg()
        self.pointmsg.point           = self.p
        """
        
        # Create the speed message.
        self.speed = 0.0
        self.speedmsg = Float64()
        self.speedmsg.data = self.speed

        # Create the sphere marker.
        self.radius_ball = 0.1
        ball_diameter = 2 * self.radius_ball

        self.p_ball_world_world = np.array([0.0, 0.0, self.radius_ball])
        self.v_ball_world_world = np.array([0.0, 0.0,  0.0            ])
        self.a_ball_world_world = np.array([0.0, 0.0, -9.81           ])

        self.ball_marker = Marker()
        self.ball_marker.header.frame_id = 'world'
        self.ball_marker.header.stamp    = self.get_clock().now().to_msg()
        self.ball_marker.action = Marker.ADD
        self.ball_marker.ns = 'ball'
        self.ball_marker.id = 1
        self.ball_marker.type = Marker.SPHERE
        self.ball_marker.pose.orientation = Quaternion()
        self.ball_marker.pose.position    = Point_from_p(self.p_ball_world_world)
        self.ball_marker.scale = Vector3(x=ball_diameter,
                                         y=ball_diameter,
                                         z=ball_diameter)
        self.ball_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)

        self.marker_array = MarkerArray(markers=[self.ball_marker])

        self.fired = False

    # Run
    def run(self):
        # Prepare Qt.
        app = QApplication(sys.argv)
        app.setApplicationDisplayName("Velocity Publisher")

        # Include a Qt Timer, setup up so every 500ms the python
        # interpreter runs (doing nothing).  This enables the ctrl-c
        # handler to be processed and close the window.
        timer = QTimer()
        timer.start(500)
        timer.timeout.connect(lambda: None)

        """# Then setup the GUI window.  And declare the ctrl-c handler to
        # close that window.
        gui = GUI_xyz(self.CB)
        signal.signal(signal.SIGINT, gui.kill)"""

        gui_speed = GUI_speed(self.CB_speed)
        gui_fire = CheckBox('Launch Ball', self.fired, self.CB_fired)

        # Layout for the GUI
        vbox = QVBoxLayout()
        vbox.addWidget(gui_speed)
        vbox.addWidget(gui_fire)
        container = QWidget()
        container.setLayout(vbox)
        container.show()

        signal.signal(signal.SIGINT, gui_speed.kill)
        #signal.signal(signal.SIGINT, gui_fire.kill)

        # Also grab the initial values, as set in the sliders.
        self.CB_speed(gui_speed.get())
        self.CB_fired(gui_fire.get())

        # Start the publisher in a separate thread.
        self.publishing = True
        thread = threading.Thread(target=self.publisherthread)
        thread.start()

        # Start the GUI window.
        self.get_logger().info("GUI starting...")
        status = app.exec_()
        self.get_logger().info("GUI ended.")

        # End the publisher.
        self.publishing = False
        thread.join()

    # Publisher Thread.
    def publisherthread(self):
        # Create a timer to control the publishing.
        rate  = self.publishrate
        timer = self.create_timer(1/float(rate), self.publish)
        dt    = timer.timer_period_ns * 1e-9
        self.get_logger().info("Publishing with dt of %f seconds (%fHz)" %
                               (dt, rate))

        # Publish until told to stop.
        while self.publishing:
            rclpy.spin_once(self)

        # Destroy the timer and report.
        timer.destroy()        
        self.get_logger().info("Stopped publishing")

    # Publish the current value.
    def publish(self):
        # Grab the current time stamp.
        stamp = self.get_clock().now().to_msg()
        # Publish the point and marker array.
        self.ball_marker.header.stamp = stamp
        """self.pointmsg.header.stamp  = stamp"""
        self.pub_mark.publish(self.marker_array)
        """self.pub_point.publish(self.pointmsg)"""
        self.pub_speed.publish(self.speedmsg)

    def update_ball(self):
        


        self.publish()

    # Callbacks
    def CB_speed(self, speed):
        self.speed = speed
        self.publish()

    def CB_fired(self, fired):
        self.fired = fired
        self.publish()

#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the GUI node (10Hz).
    rclpy.init(args=args)
    node = GUINode('speed', 10)

    # Run until interrupted.
    node.run()

    # Shutdown the node and ROS.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()