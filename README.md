Optoforce driver
===============

This driver allows you to read the data from the optoforce force sensor and publish it to redis.
it has been tested on the 6DOF force sensor only.
The data is processed to send to redis data in a direct right hand basis and with units (N for the forces, Nm for the moments)
The force given is the force that the sensor applies to the environment, in the sensor's frame.

Installation
------------
Extract the files in LINUX_API_v1.5.1.tar.gz
```sh
tar -xvf LINUX_API_v1.5.1.tar.gz
```

Create a build directory and compile using cmake

```sh
mkdir build
cd build
cmake ..
make
```

Plug the force sensor to a usb port in your computer and run the program (don't forget to run the redis driver first)
```sh
./optoforce_driver
```
