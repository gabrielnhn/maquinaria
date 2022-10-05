from flask import Flask, Response, request
from time import sleep

app = Flask(__name__)

frame = None   # global variable to keep single JPG, 
               # at start you could assign bytes from empty JPG

@app.route('/upload', methods=['PUT'])
def upload():
    global frame

    frame = request.data
    return "OK"

def gen():
    while True:
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n'
               b'\r\n' + frame + b'\r\n')
        sleep(0.04)

        
@app.route('/video')
def video():
    if frame:
        # if you use `boundary=other_name` then you have to yield `b--other_name\r\n`
        return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')
    else:
        return ""

@app.route('/')
def index():
    return 'stream:<br><img src="/video">'


if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0")

