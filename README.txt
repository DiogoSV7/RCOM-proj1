# Instruction for Serial Port Protocol

 > This folder contains the base code of the serial port protocol.

## Project Structure


> bin/: Compiled binaries.
> src/: Source code for the implementation of the link-layer and application layer protocols. Students should edit these files to implement the project.
> include/: Header files of the link-layer and application layer protocols. These files must not be changed.
> cable/: Virtual cable program to help test the serial port. This file must not be changed.
> main.c: Main file. This file must not be changed.
> Makefile: Makefile to build the project and run the application.
> penguin.gif: Example file to be sent through the serial port.

## Instructions to Run the Project


* Edit the source code in the src/ directory.
* Compile the application and the virtual cable program using the provided Makefile.
* Run the virtual cable program (either by running the executable manually or using the Makefile target):
	$ sudo ./bin/cable_app
	$ sudo make run_cable

* Test the protocol without cable disconnections and noise
	-  Run the receiver (either by running the executable manually or using the Makefile target):
		$ ./bin/main /dev/ttyS11 9600 rx penguin-received.gif
		$ make run_tx

	-  Run the transmitter (either by running the executable manually or using the Makefile target):
		$ ./bin/main /dev/ttyS10 9600 tx penguin.gif
		$ make run_rx

	-  Check if the file received matches the file sent, using the diff Linux command or using the Makefile target:
		$ diff -s penguin.gif penguin-received.gif
		$ make check_files

* Test the protocol with cable disconnections and noise
	- Run receiver and transmitter again
	- Quickly move to the cable program console and press 0 for unplugging the cable, 2 to add noise, and 1 to normal
	- Check if the file received matches the file sent, even with cable disconnections or with noise
