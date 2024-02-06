#!/usr/bin/env python3

# resources consulted:
# https://www.tutorialsteacher.com/python/create-gui-using-tkinter-python
# https://www.delftstack.com/howto/python-tkinter/how-to-change-the-tkinter-button-size/
# https://realpython.com/python-string-formatting/

from tkinter import *
import subprocess

screenwidth = 1920
screenheight = 1080
windowsize = 0.50 # 50% of fullscreen
xoffset = int((screenwidth-(windowsize*screenwidth))//2)
yoffset = int((screenheight-(windowsize*screenheight))//2)
winwidth = int(screenwidth*windowsize)
winheight = int(screenheight*windowsize)
buttonwidth = int(winwidth*0.25)
buttonheight = int(winheight*0.1)
leftbuttonposx = int((winwidth*0.25)-buttonwidth//2)
leftbuttonposy = int((winheight*0.5)-buttonheight//2)
rightbuttonposx = int(leftbuttonposx + winwidth//2)
rightbuttonposy = leftbuttonposy

def launchPeripherals():
	print("arm/roto")

def launchRover(): 
	print("rover")

if __name__ == '__main__':
	window =  Tk()
	window.title('Xbox Controller')
	window.geometry("{}x{}+{}+{}".format(winwidth, winheight, xoffset, yoffset))
	pixelVirtual = PhotoImage(width=1, height=1)
	armBtn = Button(window, text="Arm and Rototiller", font=("Helvetica", 16), 
					image=pixelVirtual, command=launchPeripherals,
					width=buttonwidth, height=buttonheight, compound="c")
	roverBtn = Button(window, text="Rover", font=("Helvetica", 16),
					  image=pixelVirtual, command=launchRover,
					  width=buttonwidth, height=buttonheight, compound="c")
	closeBtn = Button(window, text="Close", font=("Helvetica", 12),
					  image=pixelVirtual, command=window.destroy,
					  width=buttonwidth, height=buttonheight, compound="c")
	armBtn.place(x=leftbuttonposx, y=leftbuttonposy)
	roverBtn.place(x=rightbuttonposx, y=rightbuttonposy)
	closeBtn.place(x=0, y=winheight-buttonheight)
	window.mainloop()
