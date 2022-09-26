import time
from multiprocessing import Manager, Process, Queue

import curses
import npyscreen

import rospy
from std_msgs.msg import Int32, Bool, Float32, Byte


class ROSComm(Process):
    def __init__(self, terminate_queue, refresh, _dict):
        super(ROSComm, self).__init__()
        rospy.init_node("wabashui", anonymous=True)
        self.sub_pressure_upper = rospy.Subscriber("/sampler/pressure/upper/pressure", Float32, self.callback, callback_args="/sampler/pressure/upper/pressure", queue_size=1)
        self.sub_depth_upper = rospy.Subscriber("/sampler/depth/upper", Float32, self.callback, callback_args="/sampler/depth/upper", queue_size=1)
        #self.sub_depth_lower = rospy.Subscriber("/sampler/depth/lower", Float32, self.callback, callback_args="/sampler/depth/lower", queue_size=1)
        #self.sub_pressure_lower = rospy.Subscriber("/sampler/pressure/lower/pressure", Float32, self.callback, callback_args="/sampler/pressure/lower/pressure", queue_size=1)
        self.sub_temperature_upper = rospy.Subscriber("/sampler/pressure/upper/temp", Float32, self.callback, callback_args="/sampler/pressure/upper/temp", queue_size=1)
        self.sub_temperature_lower = rospy.Subscriber("/sampler/pressure/lower/temp", Float32, self.callback, callback_args="/sampler/pressure/lower/temp", queue_size=1)
        self.sub_rotaryencoder_abspos = rospy.Subscriber("/sampler/magnetic/rotary/abs_position", Int32, self.callback, callback_args="/sampler/magnetic/rotary/abs_position", queue_size=1)
        self.sub_linearencoder_abspos = rospy.Subscriber("/sampler/magnetic/linear/abs_position", Int32, self.callback, callback_args="/sampler/magnetic/linear/abs_position", queue_size=1)
        self.sub_linear_servo_vel = rospy.Subscriber("/sampler/motor/linear/read_velocity", Float32, self.callback, callback_args="/sampler/motor/linear/read_velocity", queue_size=1)
        self.sub_rotary_servo_pos = rospy.Subscriber("/sampler/motor/rotary/read_position", Int32, self.callback, callback_args="/sampler/motor/rotary/read_position", queue_size=1)
        self.sub_distance_traveled = rospy.Subscriber("/sampler/motor/linear/distance_traveled", Float32, self.callback, callback_args="/sampler/motor/linear/distance_traveled", queue_size=1)

        self.pub_anchor_left_clutch = rospy.Publisher("/usv/anchor/left/clutch", Bool, queue_size=1)
        self.pub_anchor_left_up = rospy.Publisher("/usv/anchor/left/motor/up", Bool, queue_size=1)
        self.pub_anchor_left_stop = rospy.Publisher("/usv/anchor/left/motor/stop", Bool, queue_size=1)
        self.pub_anchor_left_down = rospy.Publisher("/usv/anchor/left/motor/down", Bool, queue_size=1)
        self.pub_anchor_right_clutch = rospy.Publisher("/usv/anchor/right/clutch", Bool, queue_size=1)
        self.pub_anchor_right_up = rospy.Publisher("/usv/anchor/right/motor/up", Bool, queue_size=1)
        self.pub_anchor_right_stop = rospy.Publisher("/usv/anchor/right/motor/stop", Bool, queue_size=1)
        self.pub_anchor_right_down = rospy.Publisher("/usv/anchor/right/motor/down", Bool, queue_size=1)
        self.pub_sampler_up = rospy.Publisher("/usv/sampler/motor/up", Bool, queue_size=1)
        self.pub_sampler_stop = rospy.Publisher("/usv/sampler/motor/stop", Bool, queue_size=1)
        self.pub_sampler_down = rospy.Publisher("/usv/sampler/motor/down", Bool, queue_size=1)
        self.pub_sampler_clutch = rospy.Publisher("/usv/sampler/clutch", Bool, queue_size=1)
        self.pub_rotary_drive = rospy.Publisher("/sampler/cmd/rotary", Int32, queue_size=1)
        self.pub_linear_servo_vel = rospy.Publisher("/sampler/motor/linear/write_speed", Int32, queue_size=1)
        self.pub_solenoid = rospy.Publisher("/sampler/cmd/solenoid", Float32, queue_size=1)
        self.pub_distance_traveled_reset = rospy.Publisher("/sampler/motor/linear/distance_traveled/cmd/reset", Bool, queue_size=1)
        self.d = _dict
        self.rate = rospy.Rate(2) # Refresh view every 0.5 seconds
        self.t = terminate_queue
        self.r = refresh
        self.is_opened = True

    def terminate(self):
        self.t = True

    def callback(self, msg, topic):
        if topic in self.d:
            if msg.data == self.d[topic]:
                return
        self.d[topic] = msg.data

    def get(self, topic):
        if topic in self.d:
            return self.d[topic]
        return None

    def run(self):
        while not rospy.is_shutdown():
            if not self.t.empty():
                break
            self.r()
            self.rate.sleep()


class ProcessBar(npyscreen.Slider):
    def __init__(self, *args, **keywords):
        super(ProcessBar, self).__init__(*args, **keywords)
        self.editable = False
        self.out_of=300

class ProcessBarForRotary(npyscreen.Slider):
    def __init__(self, *args, **keywords):
        super(ProcessBarForRotary, self).__init__(*args, **keywords)
        self.editable = True
        self.step = 5
        self.out_of=179

class ProcessBarForLinear(npyscreen.Slider):
    def __init__(self, *args, **keywords):
        super(ProcessBarForLinear, self).__init__(*args, **keywords)
        self.editable = True
        self.step  = 50
        self.out_of=1000

class ProcessBarBox(npyscreen.BoxTitle):
    _contained_widget = ProcessBar

class ProcessBarBoxForRotary(npyscreen.BoxTitle):
    _contained_widget = ProcessBarForRotary

class ProcessBarBoxForLinear(npyscreen.BoxTitle):
    _contained_widget = ProcessBarForLinear

class WUSVMotorControl(npyscreen.TitleSelectOne):
    def when_value_edited(self):
        self.parent.parentApp.queue_event(npyscreen.Event("E_MOTOR", (self.name, self.value)))

class WUSVCheckBox(npyscreen.CheckBox):
    def whenToggled(self):
        self.parent.parentApp.queue_event(npyscreen.Event("E_CHECKBOX", (self.name, self.value)))

class WabashForm(npyscreen.Form):
    def create(self):
        self.add_event_hander("E_MOTOR", self.cb_e_motor)
        self.add_event_hander("E_CHECKBOX", self.cb_e_checkbox)
        # self.sub_value = rospy.Subscriber('/test/value', Int32, self.callback_value, queue_size=1)
        # self.wgconn = self.add(npyscreen.TitleText, name="ROS", value="Connected?", editable=False)
        # self.nextrely += 1

        # USV widgets
        self.wganchorleftclutch = self.add(WUSVCheckBox, name="L Clutch ON", value=True)
        self.wganchorleft = self.add(WUSVMotorControl, max_height=4, value = [1,], name="Anchor Left",
            values = ["Up","Stop","Down"], scroll_exit=True)
        self.wganchorrightclutch = self.add(WUSVCheckBox, name="R Clutch ON", value=True)
        self.wganchorright = self.add(WUSVMotorControl, max_height=4, value = [1,], name="Anchor Right",
            values = ["Up","Stop","Down"], scroll_exit=True)
        self.wgsamplerclutch = self.add(WUSVCheckBox, name="S Clutch ON", value=True)
        self.wgsampler = self.add(WUSVMotorControl, max_height=4, value = [1,], name="Sampler",
            values = ["Up","Stop","Down"], scroll_exit=True)
        # self.nextrely += 1

        # Sampler widgets
        # Pressure, temperature, and depth
        self.wgpressureupper = self.add(npyscreen.TitleText, name="Pressure(U)", value="0", editable=False)
        self.wgdepthupper = self.add(npyscreen.TitleText, name="Depth(U)", value="0", editable=False)
        #self.wgdepthlower = self.add(npyscreen.TitleText, name="Depth(L)", value="0", editable=False)
        #self.wgpressurelower = self.add(npyscreen.TitleText, name="Pressure(L)", value="0", editable=False)
        # self.wgtemperatureupper = self.add(npyscreen.TitleText, name="Temp.(U)", value="0", editable=False)
        # self.wgtemperaturelower = self.add(npyscreen.TitleText, name="Tempe.(L)", value="0", editable=False)
        self.wgdistancetraveled = self.add(ProcessBarBox, name="Distance Traveled (mm)", value=0., max_height=3)
        self.wgresetdistancetraveled = self.add(WUSVCheckBox, name="Reset distance traveled", value=False)
        # self.nextrely += 1
        # Rotary servo
        self.wgrotaryservopos = self.add(npyscreen.TitleText, name="Rotary Pos", value="0", editable=False)
        self.wgrotaryencoder = self.add(npyscreen.TitleText, name="Rotary Mag", value="0", editable=False)
        self.wgrotaryservo = self.add(WUSVCheckBox, name="Rotary Servo ON", value=False)
        self.wgrotaryangle = self.add(ProcessBarBoxForRotary, name="Angle (degree)", value=90., max_height=3)
        # self.nextrely += 1
        # Linear servo
        self.wglinearservovel = self.add(npyscreen.TitleText, name="Linear Vel", value="0", editable=False)
        self.wglinearencoder = self.add(npyscreen.TitleText, name="Linear Mag", value="0", editable=False)
        self.wglinearservo = self.add(WUSVMotorControl, max_height=4, value = [1,], name="Linear Servo",
            values = ["Up","Stop","Down"], scroll_exit=True)
        self.wglinearspeed = self.add(ProcessBarBoxForLinear, name="Speed (raw)", value=1000., max_height=3)
        self.wgsolenoid = self.add(WUSVCheckBox, name="Solenoid ON", value=False)
        # self.nextrely += 1

        self.terminate = Queue()
        self.ros = ROSComm(self.terminate, self.request_redraw, Manager().dict())
        self.ros.start()

    def cb_e_motor(self, event):
        name, selection = event.payload
        if "Anchor Left" in name:
            if 0 in selection:
                self.ros.pub_anchor_left_up.publish(True)
            elif 1 in selection:
                self.ros.pub_anchor_left_stop.publish(True)
            elif 2 in selection:
                self.ros.pub_anchor_left_down.publish(True)
        elif "Anchor Right" in name:
            if 0 in selection:
                self.ros.pub_anchor_right_up.publish(True)
            elif 1 in selection:
                self.ros.pub_anchor_right_stop.publish(True)
            elif 2 in selection:
                self.ros.pub_anchor_right_down.publish(True)
        elif "Sampler" in name:
            if 0 in selection:
                self.ros.pub_sampler_up.publish(True)
            elif 1 in selection:
                self.ros.pub_sampler_stop.publish(True)
            elif 2 in selection:
                self.ros.pub_sampler_down.publish(True)
        elif "Linear Servo" in name:
            if 2 in selection:
                # ranged in [1023, 2023]
                base_value = 1023
                v = int(base_value + self.wglinearspeed.value)
                self.ros.pub_linear_servo_vel.publish(v)
                # rospy.loginfo("linear spped (raw) is " + str(v))
            elif 1 in selection:
                self.ros.pub_linear_servo_vel.publish(0)
            elif 0 in selection:
                # ranged in [0, 1000]
                v = int(self.wglinearspeed.value)
                self.ros.pub_linear_servo_vel.publish(v)
                # rospy.loginfo("linear spped (raw) is " + str(v))

    def cb_e_checkbox(self, event):
        name, selection = event.payload
        if "S Clutch ON" in name:
            self.ros.pub_sampler_clutch.publish(selection)
        elif "R Clutch ON" in name:
            self.ros.pub_anchor_right_clutch.publish(selection)
        elif "L Clutch ON" in name:
            self.ros.pub_anchor_left_clutch.publish(selection)
        elif "Rotary Servo ON" in name:
            # rotary_drive takes integer to indicate how wide it twists
            if selection:
                max_value = 2047.
                max_angle = 179.
                r = int(self.wgrotaryangle.value * max_value / max_angle)
                
                self.ros.pub_rotary_drive.publish(r)
                # rospy.loginfo("rotary_value" + str(r))
            else:
                self.ros.pub_rotary_drive.publish(0)
        elif "Solenoid ON" in name:
            if selection:
                self.ros.pub_solenoid.publish(0.5)
            else:
                self.ros.pub_solenoid.publish(0.)
        elif "Reset distance traveled" in name:
            self.ros.pub_distance_traveled_reset.publish(True)

    def request_redraw(self):
        # print("hello")
        self.set_widget_value("/sampler/pressure/upper/pressure", self.wgpressureupper)
        self.set_widget_value("/sampler/depth/upper", self.wgdepthupper)
        #self.set_widget_value("/sampler/depth/lower", self.wgdepthlower)
        #self.set_widget_value("/sampler/pressure/lower/pressure", self.wgpressurelower)
        # self.set_widget_value("/sampler/pressure/upper/temp", self.wgtemperatureupper)
        # self.set_widget_value("/sampler/pressure/lower/temp", self.wgtemperaturelower)
        self.set_widget_value("/sampler/magnetic/rotary/abs_position", self.wgrotaryencoder)
        self.set_widget_value("/sampler/magnetic/linear/abs_position", self.wglinearencoder)
        self.set_widget_value("/sampler/motor/linear/read_velocity", self.wglinearservovel)
        self.set_widget_value("/sampler/motor/rotary/read_position", self.wgrotaryservopos)
        self.set_distance_value()
        self.display()

    def set_widget_value(self, topic, widget):
        v = self.ros.get(topic)
        if v is not None:
            widget.value = str(v)
            widget.display()

    def set_distance_value(self):
        # NOTE: 7.5 unit equals to 1 mm
        v = self.ros.get("/sampler/motor/linear/distance_traveled")
        if v is not None:
            # NOTE: Multiply by gear ratio (current gear ratio = 1:3.333333)
            self.wgdistancetraveled.value = v / 9.215336 * 3.333333 * -1
            self.wgdistancetraveled.display()

    def afterEditing(self):
        self.terminate.put(1)
        self.ros.join()
        self.parentApp.setNextForm(None)

class WabashUIApp(npyscreen.StandardApp):
    def onStart(self):
        # rospy.init_node("wabashui", anonymous=True)
        # self.sub = rospy.Subscriber("/test/int", Int32, self.)

        self.addForm("MAIN", WabashForm, name="Wabash Controller")
        # super(WabashUIApp, self).onInMainLoop = self.loop

    # def onCleanExit(self):
    #     self.ros.terminate = True
    #     self.ros.join()

if __name__ == "__main__":
    try:
        npyscreen.wrapper(WabashUIApp().run())
    except KeyboardInterrupt:
        pass
