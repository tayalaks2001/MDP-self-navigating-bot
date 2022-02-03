from logging.config import valid_ident
from constants import *
from obstacles import *
from robot import Robot, map
from tkinter import *

window = Tk()
window.title("Simulation")

canvas = Canvas(window, width = 400, height = 510)
canvas.pack()

#Set Image File to Variable
img_0 = PhotoImage(file="Image\\Image_0.png")
img_1 = PhotoImage(file="Image\\Image_1.png")

#Create and Set Label
label4_str = StringVar()
label5_str = StringVar()
label6_str = StringVar()
label4_str.set("18 1")
label5_str.set("N")
label6_str.set("0")

label1 = Label(window, text="Location :").place(x=0, y=400)
label2 = Label(window, text="Direction :").place(x=0, y=420)
label3 = Label(window, text="Turn Direction :").place(x=0, y=440)
label4 = Label(window, textvariable=label4_str).place(x=100, y=400)
label5 = Label(window, textvariable=label5_str).place(x=100, y=420)
label6 = Label(window, textvariable=label6_str).place(x=100, y=440)

#Create and Set Button
btn1 = Button(window, text="Forward", command = lambda:onclick(1)).place(x=200, y=400)
btn2 = Button(window, text="Backward", command = lambda:onclick(2)).place(x=200, y=430)
btn3 = Button(window, text="Left", command = lambda:onclick(3)).place(x=200, y=460)
btn4 = Button(window, text="Right", command = lambda:onclick(4)).place(x=200, y=490)

def draw():
    #Display the map
    x,y = 0,0
    for row in map:
        for col in row:
            if col == 0:
                canvas.create_image(x,y, anchor=NW, image=img_0)
            if col == 1:
                canvas.create_image(x,y, anchor=NW, image=img_1)
            x = x + 20
        x = 0
        y = y + 20

def update():
    #Update Information Displayed
    label4_str.set(r1.get_location())
    label5_str.set(r1.get_direction())
    label6_str.set(r1.get_turn())

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