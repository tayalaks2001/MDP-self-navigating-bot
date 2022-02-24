from distutils import command
from logging.config import valid_ident
from tracemalloc import start
from constants import *
from obstacles import *
from robot import Robot, map
from tkinter import *
import os

window = Tk()
window.title("Simulation")

canvas = Canvas(window, width = 405, height = 630)
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

obstacle_N = PhotoImage(file=os.path.join(dirname,'Image','obstacle_N.png')) #North
obstacle_E = PhotoImage(file=os.path.join(dirname,'Image','obstacle_E.png')) #East
obstacle_S = PhotoImage(file=os.path.join(dirname,'Image','obstacle_S.png')) #South
obstacle_W = PhotoImage(file=os.path.join(dirname,'Image','obstacle_W.png')) #West

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
button1 = Button(window, image = button_Up, command = lambda:onclick(1))    #Forward
button1.place(x=300, y=400)
button2 = Button(window, image = button_Down, command = lambda:onclick(2))  #Backward
button2.place(x=300, y=500)
button3 = Button(window, image = button_Left, command = lambda:onclick(3))  #Turn Left
button3.place(x=250, y=450)
button4 = Button(window, image = button_Right, command = lambda:onclick(4)) #Turn Right
button4.place(x=350, y=450) 
button1["state"] = DISABLED
button2["state"] = DISABLED
button3["state"] = DISABLED
button4["state"] = DISABLED

#Input Obstacle
label3 = Label(window, text="Obstacle x-location :").place(x=5, y=510)
label4 = Label(window, text="Obstacle y-location :").place(x=5, y=530)
label5 = Label(window, text="Obstacle direction :").place(x=5, y=550)
input1 = Entry(window, width = 5)   #InputField Obstacle x-location
input1.place(x=130, y=510)
input2 = Entry(window, width = 5)   #InputField Obstacle y-location
input2.place(x=130, y=530) 
input3 = Entry(window, width = 5)   #InputField Obstacle direction
input3.place(x=130, y=550)

command_list = ['F','F','R','F']

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
                elif col == 'ON':
                    canvas.create_image(x,y, anchor=NW, image=obstacle_N)
                elif col == 'OE':
                    canvas.create_image(x,y, anchor=NW, image=obstacle_E)
                elif col == 'OS':
                    canvas.create_image(x,y, anchor=NW, image=obstacle_S)
                elif col == 'OW':
                    canvas.create_image(x,y, anchor=NW, image=obstacle_W)  
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

def input_obstacle():
    obs_loc = [19-int(input2.get()), int(input1.get())]
    obs_dir = input3.get()
    input1.delete(0,END)
    input2.delete(0,END)
    input3.delete(0,END)
    if obs_dir == 'N':
        map[obs_loc[0]][obs_loc[1]] = 'ON' #Obstacle North
    elif obs_dir == 'S':
        map[obs_loc[0]][obs_loc[1]] = 'OS' #Obstacle South
    elif obs_dir == 'W':
        map[obs_loc[0]][obs_loc[1]] = 'OW' #Obstacle West
    else:
        map[obs_loc[0]][obs_loc[1]] = 'OE' #Obstacle East
    draw()

#Add Obstacle Button
button5 = Button(window, text="Add Obstacle", command = input_obstacle)
button5.place(x=5, y=580)

def start_sim1():
    button1["state"] = NORMAL
    button2["state"] = NORMAL
    button3["state"] = NORMAL
    button4["state"] = NORMAL
    button5["state"] = DISABLED
    button6["state"] = DISABLED

def start_sim2():
    command = command_list[0]
    if command == 'F':      #Forward Event
        button1.invoke()
    elif command == 'B':    #Backward Event
        button2.invoke()
    elif command == 'L':    #Left Event
        button3.invoke()
        button3.invoke()
        button1.invoke()
    elif command == 'R':    #Right Event
        button4.invoke()
        button4.invoke()
        button1.invoke()
    command_list.pop(0)
    window.after(1000, start_sim2)


#Start Simulation Button
button6 = Button(window, text="Start Simulation", command = start_sim1) #Start Button
button6.place(x=100, y=580)
button7 = Button(window, text="Start Simulation 2", command = start_sim2) #Start Button
button7.place(x=100, y=600)


def update():
    #Update Information Displayed
    label2_str.set(r1.get_location())

def onclick(args):
    #Clear and Update Canvas
    canvas.delete("all")
    if (args == 1) or (args == 'F'):
        #Move Up
        r1.forward()
    elif (args == 2) or (args == 'B'):
        #Move Backward
        r1.backward()
    elif (args == 3) or (args == 'L'):
        #Turn Left
        Robot.turn(r1,-1)
    elif (args == 4) or (args == 'R'):
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