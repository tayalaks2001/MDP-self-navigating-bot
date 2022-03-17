#!/usr/bin/python
import os
import os.path
import time
import serial
import logging
import json
from bluetooth import *
from threading import Thread


# Algo
from tsp import FastestPath
from maze import Maze
from constants import *
from utils import *
from generate_maze import get_random_maze_with_obstacles
from shortest_path import *
from main import Main


init = False

target_x = None
target_y = None
obstacles = None
read_target = False
facing = None
exitAlgorithm = False

detected_image = None
detected_distance = None
detected_angle = None
detected_obstacles = []

bt_server_sock = None
bt_client_sock = None
bt_buffer_size = 30


# Bluetooth Communications
bt_send_queue = []

# STM32 Communications
serial_sock = None
serial_send_queue = []
algo_navi_complete = False
time_algo_navi_complete = 0
algo_navi_adjust_state = 0
car_ready = False
car_ready_counter = 0
last_uart = ''
stitchPath = '/home/pi/stitch.txt'
filePath = '/home/pi/data.txt'
def recv_POST_data_thread():
  global detected_image
  global detected_distance
  global detected_angle
  global read_target
  global detected_obstacles
  logging.debug(f"[THREAD] Polling for POST-ed data in {filePath}")
  while True:
    if not read_target:
      time.sleep(1)
      continue
    lines = []
    try:
      if os.path.exists(filePath):
        os.remove(filePath)
        time.sleep(1)
        with open(filePath, 'r') as file:
          lines = file.readlines()
          print(lines)
        if lines and len(lines) > 0:
          try:
            detected_image, detected_distance, detected_angle = lines[0].strip().split(',')
            detected_obstacles.append(detected_image)
            read_target = False
          except ValueError as e:
            logging.error(e)
            pass
          lines = []
        os.remove(filePath)
    except Exception as e:
      logging.error(e)
    time.sleep(1)

def bluetooth_send():
  logging.debug("[THREAD] Bluetooth send thread")
  while True:
    if len(bt_send_queue) > 0:
      data = bt_send_queue[0]
      logging.debug(f"============ BLUETOOTH SEND QUEUE >> {data}")
      if bt_client_sock:
        if len(data) < bt_buffer_size:
          data = data.ljust(bt_buffer_size)
          bt_client_sock.send(data)
        elif len(data) > bt_buffer_size:
          logging.error(f"WARNING: data is longer than {bt_buffer_size} bytes")
          logging.error(f"{data}")
        else:
          bt_client_sock.send(data)
          logging.debug(data)

        bt_send_queue.pop(0)
    time.sleep(0.2)

def bluetooth_configure():
  logging.debug(f"Configuring bluetooth env")
  global bt_server_sock
  res = os.system('sudo hciconfig hci0 | grep ISCAN')

  if res != 0:
    os.system('sudo hciconfig hci0 piscan')
  
  logging.debug(f"Modified hciconfig, entering delay")
  time.sleep(2)
  logging.debug(f"Starting bluetooth socket server")
  bt_server_sock = BluetoothSocket(RFCOMM)
  bt_server_sock.bind(("",PORT_ANY))
  bt_server_sock.listen(1)
  uuid = "6c3ffa37-ac3f-454d-a114-995a8af2f2bd"
  advertise_service( bt_server_sock, "MDP-GroupOne-Server",
   service_id = uuid,
   service_classes = [ uuid, SERIAL_PORT_CLASS ],
   profiles = [ SERIAL_PORT_PROFILE ],
  # protocols = [ OBEX_UUID ]
   )
  logging.debug(f"Bluetooth server is now being advertised")

def bluetooth_runtime():
  # UP DOWN LEFT RIGHT FACING
  logging.debug("[THREAD] Bluetooth runtime thread")
  global bt_client_sock
  global client_info

  bluetooth_configure()
  logging.info(f"Waiting for connection on RFCOMM channel {bt_server_sock.getsockname()[1]}")
  while True:
    try:
      bt_client_sock, client_info = bt_server_sock.accept()
      logging.info(f"Accepted connection from {client_info}")
      try:
        while True:
          data = bt_client_sock.recv(bt_buffer_size)
          if not data or len(str(data.decode())) == 0:
            break
          bluetooth_receive_parse(data.decode())
          #bt_send_queue.append(f"Pi received: {data.decode()}")
          time.sleep(0.2)
      except IOError as e:
        logging.error(e)
      bt_client_sock.close()
    except Exception as e:
      logging.error(e)
    finally:
      if bt_client_sock:
        bt_client_sock.close()
    time.sleep(1)
  if bt_server_sock:
    bt_server_sock.close()

def bluetooth_receive_parse(data):
  global exitAlgorithm
  global serial_send_queue
  global obstacles
  logging.debug(f"[BLUETOOTH] Received [{data}]")
  if data[0:6:] == "STATE,": # grid/state update
    try:
      expectedLength = int(data.split(',')[1])
      stateData = bt_client_sock.recv(expectedLength) #Expect to receive x bytes
      if len(str(stateData.decode())) == 0:
        return
      jsonData = "{\"obstacles\":"+ stateData.decode() +"}"
      data = json.loads(jsonData)
      obstacles = []
      for i in data['obstacles']:
        obstacles.append([i['x'], i['y'], i['side']])

    except Exception as e:
      logging.error(f"Error while parsing state data {e}")
  elif data[0:5:] == "START":
    if serial_sock and car_ready:
        testThr = Thread(target=startAlgorithm, args=())
        testThr.daemon = True
        testThr.start()
  elif data[0:4:] == "STOP":
    exitAlgorithm = True
  else:
    serial_send_queue.append(data)
  
def startAlgorithm():
  global init
  global bt_send_queue
  global serial_send_queue
  global detected_distance
  global detected_angle
  global detected_image
  global read_target
  global algo_navi_complete
  global algo_navi_adjust_state
  global obstacles
  global exitAlgorithm
  #obstacles = [[2, 17, 'S'], [7, 11, 'E'], [12, 5, 'E'], [14, 15, 'S'], [19, 17, 'S']]
  if len(obstacles) < 1:
    return
  logging.debug("========== ALGORITHM START =========")
  logging.debug(obstacles)
  logging.debug(len(obstacles))

  #if init:
  #  return
  #init = True
  algo = Main([2,2,NORTH], obstacles)
  iterObstacle = 0
  pathed = []
  path = None
  target_obstacle = None
  while iterObstacle < len(obstacles):
    if exitAlgorithm:
      exitAlgorithm = False
      return
    if iterObstacle == 0 and not detected_distance and not detected_angle and iterObstacle not in pathed:
      path = algo.getPath()
      target_obstacle = algo.getTarget()
    else:
      if algo_navi_complete and detected_distance and detected_angle:
        if ' ' in detected_image and 'Arrow' not in detected_image:
          detected_image = detected_image.split(' ')[1]
        if ' ' in detected_image:
          detected_image = detected_image.replace(' ','_')
        ##obsolete ^
        bt_send_queue.append(f"SEEN,{target_obstacle[0]} {target_obstacle[1]} {detected_image}")
        path = algo.getPath(float(detected_distance), float(detected_angle))
        target_obstacle = algo.getTarget()
        time.sleep(0.2)
        detected_angle = None
        detected_distance = None
        iterObstacle = iterObstacle+1
      else:
        if algo_navi_complete and time.time() - time_algo_navi_complete > 5:
          if algo_navi_adjust_state == 0:
            algo_navi_complete = False
            serial_send_queue.append("MOVE,f")
            serial_send_queue.append("NAVICOMPLETE")
            algo_navi_adjust_state = 1
          elif algo_navi_adjust_state == 1:
            algo_navi_complete = False
            serial_send_queue.append("MOVE,b")
            serial_send_queue.append("NAVICOMPLETE")
            algo_navi_adjust_state = 2
          elif algo_navi_adjust_state == 2:
            algo_navi_complete = False
            serial_send_queue.append("MOVE,b")
            serial_send_queue.append("NAVICOMPLETE")
            algo_navi_adjust_state = 3
          elif algo_navi_adjust_state == 3:
            break

        logging.debug("Algorithm waiting for navigation to complete")
        if not detected_distance:
          logging.debug(f"Algorithm is waiting for distance and angle detection")
        time.sleep(1)
        continue
    if path and not iterObstacle in pathed:
      algo_navi_complete = False
      target = int(len(path) / 2) # For parsing returned path, path is in pairs MOTION + DISTANCE
      for i in range(0,target):
        dist = (int(path[(i * 2) + 1]) * 100)
        cmd = f"{path[(i * 2) + 0].upper()}{str(dist).zfill(4)}"
        serial_send_queue.append(cmd)
      serial_send_queue.append("NAVICOMPLETE")
      pathed.append(iterObstacle)
    time.sleep(0.2)
  try:
    write_obstacles_stitch()
  except:
    pass

def write_obstacles_stitch():
  logging.debug("Completed. Do send stitch")
  output = ""
  for i in detected_obstacles:
    output = output + i + ","
  
  output = output[0:-1:]
  with open(stitchPath, 'w+') as file:
    file.write(output)


def uart_send():
  global init
  global serial_sock
  global read_target
  global algo_navi_complete
  global time_algo_navi_complete
  logging.debug("[THREAD] UART send thread")
  while True:
    try:
      if len(serial_send_queue) > 0:
        data = serial_send_queue[0]
        if serial_sock:
          if not serial_sock.isOpen():
            serial_sock = serial.Serial("/dev/ttyUSB0", 115200)
          logging.debug(f"[UART SEND] {data}")
          command_to_send = data.encode()

          if data == "MOVE,l":
            command_to_send = b"L0000"
          elif data == "MOVE,r":
            command_to_send = b"R0000"
          elif data == "MOVE,f": #Forward 20cm
            command_to_send = b"F0200"
          elif data == "MOVE,b": #Back 20cm
            command_to_send = b"B0200"
          elif data == "MOVE,h": #Halt
            command_to_send = b"H0000"
          elif data == "NAVICOMPLETE":
            time_algo_navi_complete = time.time()
            algo_navi_complete = True
            read_target = True
          
          if data.startswith("MOVE,"):
            recalculateFacing(data.split(',')[1])
          if data != "NAVICOMPLETE":
            serial_sock.write(command_to_send)
            #pass
          serial_send_queue.pop(0)
          time.sleep(3)
        else:
          serial_sock = serial.Serial("/dev/ttyUSB0", 115200)
    except IOError as e:
      logging.error(e)
    time.sleep(0.2)

def recalculateFacing(movedDirection):
  global facing
  if movedDirection == "l":
    if facing == "N":
      facing = "W"
    elif facing == "W":
      facing = "S"
    elif facing == "S":
      facing == "E"
    elif facing == "E":
      facing = "N"
  elif movedDirection == "r":
    if facing == "N":
      facing = "E"
    elif facing == "E":
      facing = "S"
    elif facing == "S":
      facing == "W"
    elif facing == "W":
      facing = "N"

def uart_runtime():
  logging.debug("[THREAD] UART runtime thread")

  global serial_sock
  while True:
    try:
      serial_sock = serial.Serial("/dev/ttyUSB0", 115200)
      while True:
#        data = serial_sock.read(serial_sock.inWaiting())
        data = serial_sock.read(serial_sock.inWaiting())
        uart_received_data(data)
        time.sleep(0.2)
    except serial.SerialException as e:
      logging.error("SerialException")
      logging.error(e)
    except Exception as e:
      logging.error(e)
    finally:
      if serial_sock:
        serial_sock.close()
    time.sleep(0.2)

def uart_received_data(data):
  global car_ready
  global car_ready_counter
  global last_uart
  if len(data) > 0:
    latest = data[-1]
    if last_uart != latest:
      last_uart = latest
      logging.debug(f"[UART RECEIVE] {chr(latest)}")
    if chr(latest) == '1':
      car_ready_counter = car_ready_counter + 1
      if car_ready_counter > 15:
        car_ready = True
        car_ready_counter = 0
    else:
      car_ready = False


def main():
  btThr = Thread(target=bluetooth_runtime, args=())
  btThr.daemon = True
  btThr.start()

  btSendThr = Thread(target=bluetooth_send, args=())
  btSendThr.daemon = True
  btSendThr.start()

  uartThr = Thread(target=uart_runtime, args=())
  uartThr.daemon = True
  uartThr.start()

  uartSendThr = Thread(target=uart_send, args=())
  uartSendThr.daemon = True
  uartSendThr.start()

  recvPOSTdataThr = Thread(target=recv_POST_data_thread, args=())
  recvPOSTdataThr.daemon = True
  recvPOSTdataThr.start()

  try:
    while True:
      time.sleep(1)
  except KeyboardInterrupt:
    logging.info("KeyboardInterrupt received, exiting.")
    pass



if __name__ == "__main__":
  logging.basicConfig(format='%(levelname)s:\t%(message)s', level=logging.DEBUG)
  main()
