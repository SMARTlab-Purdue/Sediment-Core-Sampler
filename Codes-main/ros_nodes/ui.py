import curses
import npyscreen
from multiprocessing import Process, Queue
# import rospy
# from std_msgs.msg import Int32

class ProcessBar(npyscreen.Slider):
    def __init__(self, *args, **keywords):
        super(ProcessBar, self).__init__(*args, **keywords)
        # self.editable = False
        self.out_of=210

class ProcessBarBox(npyscreen.BoxTitle):
    _contained_widget = ProcessBar

class WabashForm(npyscreen.Form):
    # def callback_value(self, msg):
    #     self.wgconn.value = msg.data
    #     self.wgconn.update()

    def create(self):

        # self.sub_value = rospy.Subscriber('/test/value', Int32, self.callback_value, queue_size=1)
        self.wgconn = self.add(npyscreen.TitleText, name="ROS", value="Not Connected", editable=False)
        self.wgsend = self.add(npyscreen.TitleText, name="Send")
        self.wgsend.when_value_edited = curses.beep
        self.wgprog = self.add(ProcessBarBox, name="Depth (mm)", value=0, max_height=3)
        # self.wganchoron = self.add(Anchor, name="Anchor on", max_width=7, myfunc=self.hello)#, value_changed_callback=self.hello)
        # self.wganchoron.whenToggled = self.hello
        self.wganchoron = self.add(npyscreen.CheckBox, name="Anchor on")
        self.wganchoroff = self.add(npyscreen.CheckBox, name="Anchor off", check_cursor_move=False, value_changed_callback=self.hello)

        # self.grid.when_cursor_moved = curses.beep
        # rospy.init_node("wabashui")
        # self.ros_worker =

    def hello(self, widget):
        self.wganchoron.value = widget.value
        self.wganchoron.update()
        print(widget.value)

    def afterEditing(self):
        self.parentApp.setNextForm(None)

class WabashUIApp(npyscreen.NPSAppManaged):
    def onStart(self):
        self.addForm("MAIN", WabashForm, name="Wabash Controller")

    def run(self):
        print("hello")


if __name__ == "__main__":
    npyscreen.wrapper(WabashUIApp().run())
