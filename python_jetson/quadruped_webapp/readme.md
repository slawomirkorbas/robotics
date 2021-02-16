Prerequisities:

Install Flask:
	pip3 install flask


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
