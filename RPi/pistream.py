import io
import os
import picamera
import logging
import socketserver
import ctypes as ct
from picamera import mmal 
from threading import Condition
from http import server

class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_POST(self):
      if self.path == '/data':
        self.send_response(200)
        len = int(self.headers['Content-Length'])
        data = (self.rfile.read(len).decode('utf-8'))
        print(data)

        with open('/home/pi/data.txt', 'w+') as file:
          file.write(data+'\n')

    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/stream.mjpg')
            self.end_headers()
        elif self.path == "/stitch":
            sendData = []
            stitch_out_file = '/home/pi/stitch.txt'
            try:
                if os.path.exists(stitch_out_file):
                    with open(stitch_out_file, 'r') as file:
                        sendData = file.readlines()
                    self.send_response(200)
            except Exception as e:
                pass
            if len(sendData) == 0:
                sendData = ''
            else:
                sendData = sendData[0]
            self.end_headers()
            self.wfile.write(sendData.encode())
            try:
                os.remove(stitch_out_file)
            except Exception as e:
                pass
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

class PiCamera2(picamera.PiCamera):
    AWB_MODES = {
        'off':           mmal.MMAL_PARAM_AWBMODE_OFF,
        'auto':          mmal.MMAL_PARAM_AWBMODE_AUTO,
        'sunlight':      mmal.MMAL_PARAM_AWBMODE_SUNLIGHT,
        'cloudy':        mmal.MMAL_PARAM_AWBMODE_CLOUDY,
        'shade':         mmal.MMAL_PARAM_AWBMODE_SHADE,
        'tungsten':      mmal.MMAL_PARAM_AWBMODE_TUNGSTEN,
        'fluorescent':   mmal.MMAL_PARAM_AWBMODE_FLUORESCENT,
        'incandescent':  mmal.MMAL_PARAM_AWBMODE_INCANDESCENT,
        'flash':         mmal.MMAL_PARAM_AWBMODE_FLASH,
        'horizon':       mmal.MMAL_PARAM_AWBMODE_HORIZON,
        'greyworld':     ct.c_uint32(10)
        }


#with picamera.PiCamera(resolution='640x480', framerate=24) as camera:
with PiCamera2(resolution='640x480', framerate=24) as camera:
  output = StreamingOutput()
  #Uncomment the next line to change your Pi's Camera rotation (in degrees)
  #camera.rotation = 90
  #camera.awb_mode = 'greyworld' # use greyworld filter
  camera.start_recording(output, format='mjpeg')
  try:
      address = ('', 8081)
      server = StreamingServer(address, StreamingHandler)
      server.serve_forever()
  finally:
      camera.stop_recording()
