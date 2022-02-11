from logging.config import valid_ident
from constants import *
from obstacles import *
from robot import Robot, map
from tkinter import *
import os

window = Tk()
window.title("Simulation")

canvas = Canvas(window, width = 405, height = 550)
canvas.pack()

#Set Image File to Variable
dirname = os.path.dirname(__file__)
image_0 = PhotoImage(file=os.path.join(dirname,'Image','Image_0.png')) #Empty Grid
image_1 = PhotoImage(file=os.path.join(dirname,'Image','Image_1.png')) #Start Point Border

image_U = PhotoImage(file=os.path.join(dirname,'Image','Image_U.png')) #Up (Turn = 0)
image_R = PhotoImage(file=os.path.join(dirname,'Image','Image_R.png')) #Right (Turn = 1)
image_L = PhotoImage(file=os.path.join(dirname,'Image','Image_L.png')) #Left (Turn = -1)

robot_N = PhotoImage(file=os.path.join(dirname,'Image','Robot_N.png')) #North
robot_E = PhotoImage(file=os.path.join(dirname,'Image','Robot_E.png')) #East
robot_S = PhotoImage(file=os.path.join(dirname,'Image','Robot_S.png')) #South
robot_W = PhotoImage(file=os.path.join(dirname,'Image','Robot_W.png')) #West

button_Up = PhotoImage(file=os.path.join(dirname,'Image','Button_Up.png'))
button_Right = PhotoImage(file=os.path.join(dirname,'Image','Button_Right.png'))
button_Down = PhotoImage(file=os.path.join(dirname,'Image','Button_Down.png'))
button_Left = PhotoImage(file=os.path.join(dirname,'Image','Button_Left.png'))

#Create and Set Label
label2_str = StringVar()
label2_str.set("18 1")
label1 = Label(window, text="Location :").place(x=0, y=405)
label2 = Label(window, textvariable=label2_str).place(x=60, y=405)

#Create and Set Button
button1 = Button(window, image = button_Up, command = lambda:onclick(1)).place(x=300, y=400) #Forward
button2 = Button(window, image = button_Down, command = lambda:onclick(2)).place(x=300, y=500) #Backward
button3 = Button(window, image = button_Left, command = lambda:onclick(3)).place(x=250, y=450) #Turn Left
button4 = Button(window, image = button_Right, command = lambda:onclick(4)).place(x=350, y=450) #Turn Right

def draw():
    #Display the map
    x,y = 3,0
    for row in map:
        for col in row:
            if col == 0:
                canvas.create_image(x,y, anchor=NW, image=image_0)
            x = x + 20
        x = 3
        y = y + 20

    x,y = 3,0
    for row in map:
        for col in row:
            if col != 0:
                if col == 'N':
                    canvas.create_image(x-20,y-20, anchor=NW, image=robot_N)
                elif col == 'E':
                    canvas.create_image(x-20,y-20, anchor=NW, image=robot_E)
                elif col == 'S':
                    canvas.create_image(x-20,y-20, anchor=NW, image=robot_S)
                elif col == 'W':
                    canvas.create_image(x-20,y-20, anchor=NW, image=robot_W)
            x = x + 20
        x = 3
        y = y + 20

    #Display the starting point
    x,y = 2,339
    for i in range (0,60):
        canvas.create_image(i+2 ,y, anchor=NW, image=image_1)
        canvas.create_image(62,y+i, anchor=NW, image=image_1)
        canvas.create_image(62-i,399, anchor=NW, image=image_1)
        canvas.create_image(x ,399-i, anchor=NW, image=image_1)

    if r1.get_turn() == 0:
        canvas.create_image(5,435, anchor=NW, image=image_U) #Up (Turn = 0)
    elif r1.get_turn() == 1:
        canvas.create_image(5,435, anchor=NW, image=image_R) #Right (Turn = 1)
    else:
        canvas.create_image(5,435, anchor=NW, image=image_L) #Left (Turn = -1)

def update():
    #Update Information Displayed
    label2_str.set(r1.get_location())

def onclick(args):
    #Clear and Update Canvas
    canvas.delete("all")
    if args == 1:
        #Move Up
        r1.forward()
    if args == 2:
        #Move Backward
        r1.backward()
    if args == 3:
        #Turn Left
        Robot.turn(r1,-1)
    if args == 4:
        #Turn Right
        Robot.turn(r1,1)
    draw()
    update()

def main():
    window.mainloop()

if __name__ == '__main__':
    r1 = Robot()
    draw()
    main()