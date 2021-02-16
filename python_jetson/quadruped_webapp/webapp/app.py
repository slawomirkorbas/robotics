from flask import Flask, request
from flask_injector import FlaskInjector
from injector import inject
from quadruped_service import QuadrupedService
from dependencies import configure
import atexit
from signal import signal, SIGINT
from sys import exit

app = Flask(__name__)

@app.route('/')
def index():
    return 'Quadruped cat-like robot web application'


@inject
@app.route('/homePosition')
def homePosition(service: QuadrupedService):
    service.homePosition()
    return "Robot is in default position and not moving."

@inject
@app.route('/swing')
def swing(service: QuadrupedService):
    service.swing()
    return "Robot is swinging..."

@inject
@app.route('/greet')
def greet(service: QuadrupedService):
    service.greet()
    return "Robot is greeting..."

@inject
@app.route('/walk')
def walk(service: QuadrupedService):
    service.walk()
    return "Robot is walking forward..."

@inject
@app.route('/shutdown')
def shutdown_server(service: QuadrupedService):
    service.shut_down()
    func = request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    func()
    return "Service has been shut down."
    


# Setup Flask Injector, this has to happen AFTER routes are added
FlaskInjector(app=app, modules=[configure])


#if __name__ == '__main__':
#    app.run(debug=True, host='0.0.0.0')