#import sys
#sys.path.append('/services/')

from flask import Flask, request, render_template, jsonify 
from flask_injector import FlaskInjector
from injector import inject
from services.quadruped_service import QuadrupedService
from services.env_mapping_service import EnvMappingService
from dependencies import configure
import atexit
from signal import signal, SIGINT
from sys import exit
from flask_socketio import SocketIO, emit
import RPi.GPIO as GPIO
import requests
import time
from flask_script import Manager, Server
from threading import Thread, Event


app = Flask(__name__)
socketio = SocketIO(app)

@app.route('/')
def home():
    return render_template('robot-control.html')

@inject
@app.route('/stop', methods = ['POST'])
def stop(service: QuadrupedService):
    service.homePosition()
    return "Robot is in default position and not moving."

@inject
@app.route('/swing', methods = ['POST'])
def swing(service: QuadrupedService):
    service.swing()
    return "Robot is swinging..."

@inject
@app.route('/greet', methods = ['POST'])
def greet(service: QuadrupedService):
    service.greet()
    return "Robot is greeting..."

@inject
@app.route('/walk', methods = ['POST'])
def walk(service: QuadrupedService):
    service.walk()
    return "Robot is walking forward..."

@inject
@app.route('/light_test', methods = ['POST'])
def light_test(service: QuadrupedService):
    service.blink_light()
    return "Light will be blinking for 5 sec"

@inject
@app.route('/turn_right', methods = ['POST'])
def turn_right(service: QuadrupedService):
    service.turn_right()
    return "Robot is turning right..."

@inject
@app.route('/turn_left', methods = ['POST'])
def turn_left(service: QuadrupedService):
    service.turn_left()
    return "Robot is turning left..."

@inject
@app.route('/free_walk', methods = ['POST'])
def free_walk(service: QuadrupedService, env_mapping_service: EnvMappingService):
    service.free_walk_with_collision_avoidance(env_mapping_service)
    return "Robot is walking with collision avoidance..."

@inject
@app.route('/stop_free_walk', methods = ['POST'])
def stop_free_walk(service: QuadrupedService, env_mapping_service: EnvMappingService):
    service.stop_free_walk(env_mapping_service)
    return "Robot stopped walking"

@inject
@app.route('/robot_status', methods = ['GET'])
def get_robot_status(service: QuadrupedService):
    return service.get_arduino_status()

@inject
@app.route('/shutdown', methods = ['POST'])
def shutdown_server(service: QuadrupedService, env_mapping_service: EnvMappingService):
    service.shut_down()
    env_mapping_service.stop_scanning()
    # cleanup (rest) all GPIO pins
    GPIO.cleanup()
    func = request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    func()
    return "Service has been shut down."

@inject
@app.route('/start_lidar', methods = ['POST'])
def start_lidar(service: EnvMappingService):
    service.start_scanning(lidar_scan_ready_callback)
    return "Lidar started"
    

def lidar_scan_ready_callback(angle2dist):
    # Send the data in a websocket event
    socketio.emit('lidar_scan_ready', {'lidarSamples': angle2dist })

@inject
@app.route('/lidar_sample', methods = ['GET'])
def get_lidar_sample(service: EnvMappingService):
    return jsonify(service.scan())

@inject
@app.route('/stop_scan', methods = ['POST'])
def stop_scan(service: EnvMappingService):
    service.stop_scanning()
    return "Lidar stopped"
    
@socketio.on('test event')
def test_event(msg):
  print (msg['data'])
@socketio.on('connect')
def on_connect():
  emit('connect_confirm',{'status':'CONNECTED'})
@socketio.on('disconnect')
def on_connect():
  print ('Client disconnected!')


def toggle_robot_walking_state(channel):
    global robot_is_walking
    if(robot_is_walking == False):
        res = requests.post('http://localhost:5000/free_walk')
        robot_is_walking = True
    else:
        if(robot_is_walking == True):
            res = requests.post('http://localhost:5000/stop_free_walk')
            robot_is_walking = False
    setup_push_button_action()

def setup_push_button_action():
    GPIO.cleanup(pull_up_button_pin)
    GPIO.cleanup(start_button_pin)
    GPIO.setup(pull_up_button_pin,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(start_button_pin,GPIO.IN)
    GPIO.add_event_detect(start_button_pin, GPIO.RISING)
    GPIO.add_event_callback(start_button_pin, toggle_robot_walking_state)
    

FlaskInjector(app=app, modules=[configure])
GPIO.setmode(GPIO.BOARD)
pull_up_button_pin = 36
start_button_pin = 38
robot_is_walking = False
setup_push_button_action()


if __name__ == '__main__':
    # Setup Flask Injector, this has to happen AFTER routes are added
    app.run(debug=True, host='0.0.0.0')
