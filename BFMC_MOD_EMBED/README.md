# IMPORTANT NOTES FROM BEHZAD

In RemoteControlTransmitterProcess.py
self.serverIp should be car ip add

In main.py should be
enableStream        =  False
enableCameraSpoof   =  False 
enableRc            =  True

Then car embbeded board should be connected to a usb port on jetson
jetson power at thıs moment should be a plug power and seprated from embedded board power supply

If you run on the error of permission-denied-on /dev/ttyacm0 
just run this code

sudo chmod a+rw /dev/ttyACM0

On the jetson 
pip3 install -r requirements_rpi.txt

then on the car run python3 main.py in the folder of main.py

and on the remote pc you should first run the 
pip3 install -r requirements_remote.txt

Note I did not modify so much in the codes just it was for verfication of simple control.
Also use the files in this folder for pc and jetson just modify ip address.




# BFMC - Brain Project

The project contains all the provided code for the RPi, more precisely:
- Firmware for communicating with the Nucleo and control the robot movements (Speed with constant current consumption, speed with constant speed, braking, moving and steering);
- Firmware for gathering data from the sensors (IMU and Camera);
- API's for communicating with the environmental servers at Bosch location;
- Simulated servers for the API's.

## The documentation is available in more details here:
[Documentation](https://boschfuturemobility.com/brain/)
