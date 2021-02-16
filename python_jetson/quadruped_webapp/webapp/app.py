from flask import Flask
from flask_injector import FlaskInjector
from injector import inject
from quadruped_service import QuadrupedService
from dependencies import configure

app = Flask(__name__)

@app.route('/')
def index():
    return 'Quadruped cat-like robot web application'


@inject
@app.route('/homePosition')
def homePosition(service: QuadrupedService):
    return service.homePosition()
    

# Setup Flask Injector, this has to happen AFTER routes are added
FlaskInjector(app=app, modules=[configure])


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')