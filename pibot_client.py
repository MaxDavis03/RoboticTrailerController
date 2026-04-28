#!/usr/bin/env python
import time
import requests
import sys
from threading import Thread
import json

import cv2
import numpy as np
import argparse

import signal
import multiprocessing as mp

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def print_coloured(text:str,colour_code:str,*args,**kwargs):
    try:
        print(colour_code + text + bcolors.ENDC,*args,**kwargs)
    except KeyboardInterrupt as e:
        raise KeyboardInterrupt
    except Exception as e:
        # For OS's that don't support the control characters.
        print(text,*args,**kwargs)

class VideoStreamWidget(object):
    def __init__(self, src:int|str=0,refresh_rate:float=0.01):
        self.src = src
        self.refresh_rate = refresh_rate
        self.capture = cv2.VideoCapture(src)
        self._frame = None
        if not self.capture.isOpened():
            # Attempt to open the capture with a different API. Mainly just to
            # get around GStreamer.
            self.capture.release()
            self.capture = cv2.VideoCapture(src,apiPreference=cv2.CAP_FFMPEG)
        
        self._stream_available = self.capture.isOpened()

        if self._stream_available:
            print_coloured('Opened capture, start thread',bcolors.OKGREEN)
            # Start the thread to read frames from the video stream
            self.thread = Thread(target=self.update, args=())
            self.thread.daemon = True
            self._running = mp.Value('b',True)
            self.thread.start()
        else:
            self.capture.release()
            print_coloured("Failed to open streamed capture.",bcolors.WARNING)
            try:
                img = None
                for attempt_num in range(10):
                    img = self.frame
                    if isinstance(img,np.ndarray):
                        break
                    print_coloured(f"Could not grab image. Attempt {attempt_num+1}/{10}",bcolors.WARNING)
                if not isinstance(img,np.ndarray):
                    print_coloured("Could not capture image.",bcolors.FAIL)
            except KeyboardInterrupt as e:
                raise e
            except Exception as e:
                print_coloured("Could not capture image.",bcolors.FAIL)
        

    def update(self):
        # Read the next frame from the stream in a different thread
        while self._running.value:
            if self.capture.isOpened():
                (self.status, self._frame) = self.capture.read()
            time.sleep(self.refresh_rate)

    def show_frame(self):
        # Display frames in main program
        cv2.imshow('frame', self.frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.capture.release()
            cv2.destroyAllWindows()
            exit(1)
    
    def release(self):
        self._running.value = False
        self.thread.join()
        self.capture.release()

    @property
    def frame(self):
        if self._stream_available:
            return self._frame
        elif isinstance(self.src,str) and self.src.startswith("http"):
            # Manually grab and decode the byte stream to get around cv2 failing
            jpeg_buff = requests.get(self.src,params={"dummy":True}).content
            numpy_jpeg = np.frombuffer(jpeg_buff,dtype=np.uint8)
            frame = cv2.imdecode(numpy_jpeg,cv2.IMREAD_COLOR)
            return frame
            
class PiBot(object):
    def __init__(self, ip='localhost', port=8080, localiser_ip=None, localiser_port=8080):
        self.ip = ip
        self.port = port
        self.localiser_ip = localiser_ip
        self.localiser_port = localiser_port
        self.localiser_endpoint = None

        self._signals = [signal.SIGINT,signal.SIGTERM]
        self._original_sig_handlers = [signal.getsignal(val) for val in self._signals]
        self._enable_signals()

        self.endpoint = 'http://{}:{}'.format(self.ip, self.port)
        if localiser_ip is not None:
            self.localiser_endpoint = 'http://{}:{}'.format(localiser_ip, localiser_port)

            num_attempts = 10
            for attempt_num in range(num_attempts):
                try:
                    requests.get('{}/pose/get?group={}'.format(self.localiser_endpoint, 0), timeout=1)
                    print_coloured('Localiser setup',bcolors.OKGREEN)
                    break
                except requests.exceptions.Timeout as e:
                    print_coloured(f"Failed to communicate with localiser ({attempt_num+1}/{num_attempts}).",bcolors.WARNING)
                    continue
                except requests.ConnectionError as e:
                    print_coloured("Could not connect to Localiser.",bcolors.FAIL)
                    break
                print_coloured("Could not connect to Localiser.",bcolors.FAIL)
        else:
            print_coloured('Localiser was not setup.',bcolors.WARNING)

        self.camera = VideoStreamWidget('{}/camera/get'.format(self.endpoint))
        # if self.camera.available:
        print('Wait for first camera image')
        while self.camera.frame is None:
            time.sleep(0.1)
        print_coloured("Acquired image from camera.",bcolors.OKGREEN)

    def __del__(self):
        # Stop motors and close connection
        try:
            requests.get('{}/robot/stop'.format(self.endpoint), timeout=1,headers={"Connection":"close"})
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port),bcolors.WARNING, file=sys.stderr)
        except requests.ConnectionError as e:
            # No connection to close
            pass

        # Close any connection to the localiser.
        try:
            if self.localiser_endpoint is not None:
                requests.get('{}/pose/get?group={}'.format(self.localiser_endpoint, 0), timeout=1, headers={"Connection":"close"})
        except requests.exceptions.Timeout as e:
            print_coloured('Timed out attempting to communicate with {}:{}'.format(self.localiser_ip, self.localiser_port),bcolors.WARNING,file=sys.stderr)
        except requests.ConnectionError as e:
            # No connection to close
            pass

        if hasattr(self,"camera"):
            self.camera.release()


    def _handle_signals(self,sig,context):
        self.__del__() # Doesn't actually delete the object.
        self._clear_signals()
        signal.raise_signal(sig)

    def _enable_signals(self):
        for sig,orig_func in zip(self._signals,self._original_sig_handlers):
            if signal.getsignal(sig) == orig_func:
                signal.signal(sig,lambda *args,**kwargs: self._handle_signal(*args))
    def _clear_signals(self):
        for sig,orig_func in zip(self._signals,self._original_sig_handlers):
            signal.signal(sig,orig_func)
        

    def setVelocity(self, motor_left=0, motor_right=0, duration=None, acceleration_time=None):
        try:
            params = [
                'value={},{}'.format(motor_left, motor_right)
            ]

            if duration is not None:
                assert duration > 0, 'duration must be positive'
                assert duration < 20, 'duration must be < network timeout (20 seconds)'

                params.append('time={}'.format(duration))
            
                if acceleration_time is not None:
                    assert acceleration_time < duration / 2.0, 'acceleration_time must be < duration/2'
                    params.append('accel={}'.format(acceleration_time))

            # print('{}/robot/set/velocity?{}'.format(self.endpoint, '&'.join(params)))
            resp = requests.get('{}/robot/set/velocity?{}'.format(self.endpoint, '&'.join(params)))
            
            if resp.status_code != 200:
                raise Exception(resp.text)

            return resp.json()
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return None

    def setLED(self, number, state):
        try:
            assert number >= 2 and number <= 4, 'invalid LED number'
          
            requests.get('{}/led/set/state?id={}&value={}'.format(self.endpoint, number, 1 if bool(state) else 0))
            return True
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False

    def pulseLED(self, number, duration):
        try:
            assert number >= 2 and number <= 4, 'invalid LED number'
            assert duration > 0 and duration <= 0.255, 'invalid duration'
          
            requests.get('{}/led/set/count?id={}&value={}'.format(self.endpoint, number, duration * 1000))
            return True

        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False

    def getDIP(self):
        try:
            resp = requests.get('{}/hat/dip/get'.format(self.endpoint))
            return int(resp.text)
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False
    
    def getButton(self):
        try:
            resp = requests.get('{}/hat/button/get'.format(self.endpoint))
            return int(resp.text)
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False
   
    def setLEDArray(self, value):
        try:
            requests.get('{}/hat/ledarray/set?value={}'.format(self.endpoint, int(value)))
            return True
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False

    def printfOLED(self, text, *args):
        try:
            requests.get('{}/hat/screen/print?value={}'.format(self.endpoint, text % args))
            return True
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False

    def setScreen(self, screen):
        try:
            requests.get('{}/hat/screen/set?value={}'.format(self.endpoint, int(screen)))
            return True
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False
    
    def stop(self):
        try:
            resp = requests.get('{}/robot/stop'.format(self.endpoint), timeout=1)
            return resp.json()
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return None

    def resetPose(self):
        try:
            resp = requests.get('{}/robot/pose/reset'.format(self.endpoint), timeout=5)
            return True
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False

    def resetEncoder(self):
        try:
            resp = requests.get('{}/robot/hw/reset'.format(self.endpoint), timeout=5)
            return True
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False

    def getImage(self):
        return self.camera.frame

    def getVoltage(self):
        try:
            resp = requests.get('{}/battery/get/voltage'.format(self.endpoint), timeout=1)
            return float(resp.text) / 1000
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return None
    
    def getCurrent(self):
        try:
            resp = requests.get('{}/battery/get/current'.format(self.endpoint), timeout=1)
            return float(resp.text) / 1000
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return None
        
    def getEncoders(self):
        try:
            resp = requests.get('{}/robot/get/encoder'.format(self.endpoint), timeout=1)
            left_enc, right_enc = resp.text.split(",")
            return int(left_enc), int(right_enc)
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return None

    def getLocalizerImage(self):
        if self.localiser_endpoint is None:
            print('No localiser endpoint specified')
            return None
        try:
            resp = requests.get('{}/camera/get'.format(self.localiser_endpoint), timeout=1)
            img = np.frombuffer(resp.content, dtype=np.uint8)
            img = cv2.imdecode(img, cv2.IMREAD_COLOR)
            return img
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.localiser_ip, self.localiser_port), file=sys.stderr)
            return None

    def getLocalizerPose(self, group_number = 14):
        if self.localiser_endpoint is None:
            print('No localiser endpoint specified')
            return None
        try:
            resp = requests.get('{}/pose/get?group={}'.format(self.localiser_endpoint, group_number), timeout=1)
            json_decoded = json.loads(resp.text)
            x, y, theta = json_decoded['pose']['x'], json_decoded['pose']['y'], json_decoded['pose']['theta']
            return float(x), float(y), float(theta)
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.localiser_ip, self.localiser_port), file=sys.stderr)
            return None
 

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='PiBot client')
    parser.add_argument('--ip', type=str, default='localhost', help='IP address of PiBot')
    parser.add_argument('--port', type=int, default=8080, help='Port of PiBot')
    parser.add_argument('--localiser-ip', type=str, default='egb439localiser1', help='IP address of localiser', required=False)
    parser.add_argument('--localiser-port', type=int, default=8080, help='Port of localiser', required=False)
    parser.add_argument('--group_num', type=int, default=None, help='Group number', required=False)
    args = parser.parse_args()

    bot = PiBot(args.ip, args.port, args.localiser_ip, args.localiser_port)
    img = bot.getImage()
    print("robot image size %d by %d" % (img.shape[0], img.shape[1]))

    if bot.localiser_endpoint is not None:
        localizer_img = bot.getLocalizerImage()
        print("localiser image size %d by %d" % (localizer_img.shape[0], localizer_img.shape[1]))

        robot_x, robot_y, robot_theta = bot.getLocalizerPose(args.group_num)
        if robot_x == 0 and robot_y == 0 and robot_theta == 0:
            print("robot was not found by localizer")
        else:
            print("robot is currently at: x=%.2f y=%.2f theta=%.2f" % (robot_x, robot_y, robot_theta))

    # bot.setVelocity(-50, -50)
    # time.sleep(2)
    # bot.stop()

    # bot.setVelocity(-50, 50, duration=2)
    # bot.setVelocity(100, 100, 7, 3)

    print(f'Voltage: {bot.getVoltage():.2f}V')
    print(f'Current: {bot.getCurrent():.2f}A')
    encs = bot.getEncoders()
    print(f'Encoder left: {encs[0]}, right: {encs[1]}')

    cv2.imshow('image', img)
    cv2.waitKey(0)

    bot.resetEncoder()
