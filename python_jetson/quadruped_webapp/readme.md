Prerequisities:

Install Libs:
	pip3 install flask
	pip3 install Flask-SocketIO
	pip3 install pySerial
	sudo pip install Jetson.GPIO
	sudo apt install nodejs
	
Enable GPIO pins on Jetson Nano
	sudo groupadd -f -r gpio
	sudo usermod -a -G gpio your_user_name
	


To run app:

upload 'arduino_mcu_quadruped_program.ino' using Arduino IDE

Run Flask quadruped_webapp by:

	flask run

Optionally from 'quadruped_webapp' directory run:

	python -m flask run


access endpoints:

	http://localhost:5000/shutdown
	http://localhost:5000/homeposition
	http://localhost:5000/swing
