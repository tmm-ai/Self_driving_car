# Self-Driving Program for Picar Model Car

## Introduction
This project was developed for CS437 Internet of Things at the University of Illinois Urbana-Champaign (UIUC) as part of my
Masters in Computer Science. The Project team 
members were Amaarah Johnson, Jamie Stephens, and Tom McNulty.

We worked with the SunFounder Raspberry Pi Car Robot Kit, 4WD HAT Module with Ultrasonic Sensor, and installed the
Raspberry Pi camera version 2.1 to the front of the car. Our objective was to create a self-driving program that could do the following: 
1)  Scan its immediate area with an ultrasonic 
sensor and determine where all the obstacles are within a 1-meter radius, 
2) Create a map that includes the car's location, the obstacles' layout, and the final destination.
3) Define an optimal route through the obstacles to the final destination
4) Appropriately respond (e.g., stopping) when encountering traffic signs and people. 
5) Communicate with the user via wifi and Bluetooth to relay the map, the optimal route, and car information such as distance traveled, current direction, and temperature of raspberry pi CPU.

The code in these files only provides self-navigation capabilities. See the requirements to get the modules for simple car operations. 

## Modules Provided

1) Navigation.py is the primary driver of the navigation system and is where the user can adjust the configuration variables, 
starting on line 363. 
2) A_star.py calculates the optimal route from start to finish around all the obstacles. Not surprisingly, it uses
the A* algorithm to do this. 
3) obj_detection.py is for the camera to identify traffic signs and humans. 
4) The car maker, SunFounder, provided __init__.py and is included in our code as a convenience to see the functions called from the Navigation.py module easily.

## Requirements

The code for basic Sunfounder car functionality, such as driving, turning, and utilizing the ultrasonic sensor, can be found at:
https://github.com/sunfounder/picar-4wd

How to connect and get started with the pi-camera version 2 can be found here:
https://projects.raspberrypi.org/en/projects/getting-started-with-picamera/0

## Installation

Once you have the car, follow the instructions to install the raspberry pi OS onto a micro SD, then
install the Sunfounder car code on the micro SD. 

The basic car code can be found here:  https://github.com/sunfounder/picar-4wd
You can copy the car naviation code modules into the picar-4wd/picar_4wd folder. The provided files will need to be in the same folder as the  __init__.py module. 

## Configuration

In module navigation.py, starting on line 363, you will see a section titled "Setting Basic Parameters." Here you can
choose how large of an area you want for your driving area/obstacle course. The dim_x and dim_y variables establish the driving area, which the code will use to create the 2D grid/map. The grid_condenser indicates how many centimeters you would like each unit to be. grid_condenser is set at 5cm, and the dimensions are 50x50, thus making a square 2.5-meter mapping/driving
area. The car is oriented at the bottom of the grid, facing upwards. Therefore on the grid, the car is in the middle column in the last row. The end_row and end_col set the designated destination on the grid.

The car does not continuously use the ultrasonic scanner as it is driving. We assume the final destination will be 
along the top row (row 0). The second ultrasonic scan will occur once it is at or above row 34 and
facing upward (its initial direction). It will then stop, do a new scan, develop a new map of its environment, create a 
new optimal path, and proceed along that path. 

The car currently scans for humans and traffic signs via the camera. The vehicle will pause about every 2 seconds to conduct
this scan and stops as needed for humans and stop signs. There are no easily adjustable settings to change this 
optical scanning at this time. 

## Maintainers
@TomMcOO1 on Twitter




