from flask import Flask, render_template, Response, jsonify, request, send_from_directory, send_file
from camera import VideoCamera
import sys

app = Flask(__name__, static_folder='static')


last_video_file = None
video_camera = None
global_frame = None
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/record_status', methods = ['POST'])
def record_status():
    print("record status was called!!!!!!!!!!!!!!!")
 
    global video_camera
    global last_video_file
    if video_camera == None:
        video_camera = VideoCamera()

    json = request.get_json()

    status = json['status']

    if status == "true":
        video_camera.start_record()
        return jsonify(result="started")
    else:
        video_camera.stop_record()
        last_video_file = video_camera.collect_time()
        print("last video file was just set!")
        print(last_video_file)
        return jsonify(result="stopped")

def video_stream():
    global video_camera 
    global global_frame

    if video_camera == None:
        video_camera = VideoCamera()
        
    while True:
        frame = video_camera.get_frame()

        if frame != None:
            global_frame = frame
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
        else:
            yield (b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n\r\n' + global_frame + b'\r\n\r\n')
import os


@app.route('/download_video')
def download_video():
    global video_camera
    return send_from_directory(app.static_folder, last_video_file+'.avi')

@app.route('/video_viewer')
def video_viewer():
    return Response(video_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', threaded=True)
