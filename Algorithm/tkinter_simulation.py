from logging.config import valid_ident
from constants import *
from obstacles import *
from robot import Robot, map
from tkinter import *
import os

window = Tk()
window.title("Simulation")

canvas = Canvas(window, width = 405, height = 510)
canvas.pack()

#Set Image File to Variable
dirname = os.path.dirname(__file__)
img_0 = PhotoImage(file=os.path.join(dirname,'Image','Image_0.png'))
img_1 = PhotoImage(file=os.path.join(dirname,'Image','Image_1.png')) #North
img_2 = PhotoImage(file=os.path.join(dirname,'Image','Image_2.png')) #East
img_3 = PhotoImage(file=os.path.join(dirname,'Image','Image_3.png')) #South
img_4 = PhotoImage(file=os.path.join(dirname,'Image','Image_4.png')) #West
img_blank = PhotoImage(file=os.path.join(dirname,'Image','Image_blank.png'))
img_U = PhotoImage(file=os.path.join(dirname,'Image','Image_U.png')) #Up (Turn = 0)
img_R = PhotoImage(file=os.path.join(dirname,'Image','Image_R.png')) #Right (Turn = 1)
img_L = PhotoImage(file=os.path.join(dirname,'Image','Image_L.png')) #Left (Turn = -1)


#Create and Set Label
label2_str = StringVar()
label2_str.set("18 1")

label1 = Label(window, text="Location :").place(x=0, y=405)
label2 = Label(window, textvariable=label2_str).place(x=60, y=405)

#Create and Set Button
btn1 = Button(window, text="Forward", command = lambda:onclick(1)).place(x=200, y=400)
btn2 = Button(window, text="Backward", command = lambda:onclick(2)).place(x=200, y=430)
btn3 = Button(window, text="Left", command = lambda:onclick(3)).place(x=200, y=460)
btn4 = Button(window, text="Right", command = lambda:onclick(4)).place(x=200, y=490)

def draw():
    #Display the map
    x,y = 3,0
    for row in map:
        for col in row:
            if col == 0:
                canvas.create_image(x,y, anchor=NW, image=img_0)
            elif col == 'N':
                canvas.create_image(x,y, anchor=NW, image=img_1)
            elif col == 'E':
                canvas.create_image(x,y, anchor=NW, image=img_2)
            elif col == 'S':
                canvas.create_image(x,y, anchor=NW, image=img_3)
            elif col == 'W':
                canvas.create_image(x,y, anchor=NW, image=img_4)
            x = x + 20
        x = 3
        y = y + 20

    #Display the starting point
    x,y = 2,339
    for i in range (0,60):
        canvas.create_image(i+2,y, anchor=NW, image=img_blank)
        canvas.create_image(62,y+i, anchor=NW, image=img_blank)
        canvas.create_image(62-i,399, anchor=NW, image=img_blank)
        canvas.create_image(x,399-i, anchor=NW, image=img_blank)

    if r1.get_turn() == 0:
        canvas.create_image(5,435, anchor=NW, image=img_U) #Up (Turn = 0)
    elif r1.get_turn() == 1:
        canvas.create_image(5,435, anchor=NW, image=img_R) #Right (Turn = 1)
    else:
        canvas.create_image(5,435, anchor=NW, image=img_L) #Left (Turn = -1)

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