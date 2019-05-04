import cv2
import threading
import datetime


class RecordingThread (threading.Thread):
    def __init__(self, name, camera):
        threading.Thread.__init__(self)
        self.name = name
        self.isRunning = True

        self.cap = camera
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('./static/video.avi',fourcc, 20.0, (640,480))

    def run(self):
        cv2.imwrite("TEST.jpg", self.cap.read()[1])
        while self.isRunning:
            ret, frame = self.cap.read()
            if ret:
                self.out.write(frame)

        self.out.release()

    def stop(self):
        self.isRunning = False

    def __del__(self):
        self.out.release()

class VideoCamera(object):
    def __init__(self):
        # Open a camera
        self.cap = cv2.VideoCapture(0)
      
        # Initialize video recording environment
        self.is_record = False
        self.out = None

        # Thread for recording
        self.recordingThread = None

        # Recording time variable
        self.recording_time = None
    
    def __del__(self):
        self.cap.release()
    
    def get_frame(self):
        ret, frame = self.cap.read()

        if ret:
            ret, jpeg = cv2.imencode('.jpg', frame)

            # Record video
            if self.is_record:
                if self.out == None:
                    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                    data = datetime.datetime.now()
                    self.recording_time = data.strftime("%Y_%m_%d_%H_%M_%S")
                    self.out = cv2.VideoWriter('./static/' + self.recording_time + '.avi',fourcc, 20.0, (640,480))
                
                ret, frame = self.cap.read()
                if ret:
                    self.out.write(frame)
            else:
                if self.out != None:
                    self.out.release()
                    self.out = None  

            return jpeg.tobytes()
      
        else:
            return None

    def collect_time(self):
        return self.recording_time

    def start_record(self):
        self.is_record = True
        #self.recordingThread = RecordingThread("Video Recording Thread", self.cap)
        #self.recordingThread.start()

    def stop_record(self):
        self.is_record = False

        #if self.recordingThread != None:
        #    self.recordingThread.stop()

            
