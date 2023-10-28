# Fall Detection Project ReadMe
This project is a fall detection system that utilizes wearable sensors and a signal processing algorithm to detect falls in real-time. The source code files in this project are designed to work together to create a fall detection system that can be integrated into other applications

## Source Code Files
`fall.py`: This is the main file of the project that runs on the Raspberry Pi. It reads accelerometer data sent via Bluetooth using the `receive_data` function and processes it using the `process_data` function. The file also includes utility functions used by the data acquisition and signal processing segments of the code. These functions include a recursive moving average function, thresholding feature vectors, and sending emergency alerts.

`teensy_data_acquisition.ino`: This file contains the code for data acquisition. It reads raw accelerometer sensor data and sends its corresponding 'g' value to the Raspberry Pi module using the HC-05 Bluetooth communication module.

## How to Use
To use this project, you will need to upload the teensy_data_acquistion.ino file onto the Teensy microcontroller and hook it up to a power supply. 
Next perform the following tasks on the R-Pi to connect with the teensy unit via Bluetooth:
•	Go to the Applications Menu→Preferences→Bluetooth Manager to open up the Bluetooth manager window.
•	Click on Search, and a list of bluetooth devices nearby should show up. Select the one called HC-05. If none are labeled as HC-05, choose the device with MAC address starting with 98:xx:xx:xx:xx:xx.
•	Once the device is highlighted, go to Device and click on “Trust”
•	Click on “Pair” in the Device tab. This will open a Pairing request pop-up asking for PIN authentication. Enter in the PIN (1234 by default).
•	The LED on the HC-05 should start blinking at a much slower rate (∼ every 2 seconds), indicating that the pairing was successful. You can now exit the “Pairing” window if it doesn’t close automatically.
•	Now with the HC-05 selected on the Bluetooth manager window. Click on the Device→Connect to:
Serial Port. You should get a pop-up message that looks similar to:
Serial Port service on device HC-05 now will be available via /dev/rfcomm0
Open fall.py and ensure that the Serial Port is the same as the one indicated by the Bluetooth manager (Ex: /dev/rfcomm0). Next open a terminal window and go to the directory that contains the fall.py file. Once there run the command: python fall.py -imu to begin running the fall detection algorithm.

## Theory of Operation
The fall detection system comprises two units: the Teensy data acquisition unit and the Raspberry Pi data processing unit. The user's wearable sensor mainly consists of the Teensy data acquisition board, which collects the user's acceleration measurements in 'g' units and sends them to the Raspberry Pi for data and signal processing.
The Raspberry Pi operates on a multi-threaded configuration, with one thread 'listening' for incoming data from the Teensy and another thread for processing the data. The processing thread filters incoming data, extracts features, detects falls based on thresholds, and sends alerts when a fall is detected.
When a fall is detected, the system alerts the user interface (in our case, a terminal screen) to indicate that a fall has occurred. The Raspberry Pi then sends an emergency alert to notify caregivers or emergency services. If the user double taps the wearable sensor within 10 seconds after the fall has been detected, the emergency alert protocol is cancelled.
Overall, the fall detection system provides an effective and reliable way to detect falls in real-time, which can help prevent injuries and save lives.
