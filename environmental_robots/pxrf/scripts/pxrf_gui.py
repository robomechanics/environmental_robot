#!/usr/bin/env python3
import time
from tkinter import *
import rospy
from std_msgs.msg import String
import sys

# add pxrf's plot script to lookup path
import os
import rospkg
rospack = rospkg.RosPack()
pxrf_path = rospack.get_path('pxrf')
sys.path.insert(0, os.path.abspath(os.path.join(pxrf_path, "scripts")))

from plot import generate_plot
from sensor_msgs.msg import NavSatFix

class App:
    def __init__(self, window):
        self.window = window
        self.window.title("PXRF Contoller")

        self.width = 800
        self.height = 600
        self.xoffset = 200
        self.yoffset = 200

        self.window.geometry(f"{self.width}x{self.height}+{self.xoffset}+{self.yoffset}")
        self.buttonwidth = int(self.width*0.25)
        self.buttonheight = int(self.height*0.1)
        self.safety = Label(root, text="Operation Safety: \n 1) drive disabled if pxrf is low on the ground \n 2) Hold B to operate rake and pxrf \n 3) In case of malfunction, use the emergency stop / press CTRL + C ", font=("Courier", 14))
        self.safety.place(relx = 0.5, rely = 0.1, anchor = 'center')
        
        self.pxrfLabel = Label(root, text="PXRF Test Not Running", font=("Helvetica", 24))
        self.gpsLabel = Label(root, text="Searching for gps signal...", font=("Helvetica", 18))
        self.pxrfLabel.place(relx = 0.5, rely = 0.40, anchor = 'center')
        self.gpsLabel.place(relx = 0.5, rely = 0.75, anchor = 'center')

        self.pixelVirtual = PhotoImage(width=1, height=1)
        self.pxrfButton = Button(window, text="Start", font=("Helvetica", 16), image=self.pixelVirtual, 
                                command=self.toggle_pxrf, width=self.buttonwidth, height=self.buttonheight, compound="c")
        self.pxrfButton.place(relx = 0.5, rely = 0.5, anchor = 'center')

        # initialize node
        rospy.init_node('pxrf_gui', anonymous=True)
        self.pubCTRL = rospy.Publisher('pxrf_cmd', String, queue_size=10)
        self.subCTRL = rospy.Subscriber("pxrf_response", String, self.listener)
        self.gps = rospy.Subscriber("/gnss1/fix", NavSatFix, self.location)
        # add sleep after button press (prevent overclicks)
        self.rate = rospy.Rate(1)

        self.pxrfRunning = False
        
    def listener(self, msg):
        if msg.data == "201":
            self.pxrfRunning = False
            self.pxrfLabel.config(text="PXRF Test Not Running")
            self.pxrfButton.config(text="Start PXRF Test")
           # self.rate.sleep()
            #time.sleep(2)
            #generate_plot()

    def location(self,msg):
        if (msg.longitude != None and msg.latitude != None and msg.latitude != 0 and msg.longitude != 0):
            self.gpsLabel.config(text="GPS found")
		     
    def toggle_pxrf(self):
        if not self.pxrfRunning:
            print("PXRF Test Started!")
            self.pxrfLabel.config(text="PXRF Test Running!")
            self.pxrfRunning = True
            self.pxrfButton.config(text="Stop and Plot the data")
            self.pubCTRL.publish("start")
        else:
            print("PXRF Test Stopped")
            self.pxrfLabel.config(text="PXRF Test Not Running")
            self.pxrfRunning = False
            self.pxrfButton.config(text="Start PXRF Test")
            self.pubCTRL.publish("stop")
            #time.sleep(2)
            #generate_plot()
        # self.rate.sleep()


if __name__ == '__main__':
    root = Tk()
    my_gui = App(root)
    root.mainloop()
