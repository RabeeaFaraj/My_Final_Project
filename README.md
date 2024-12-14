1.The project focuses on the development of an autonomous system for identifying and deterring unauthorized mobile targets based on their color.
The system:
- Is based on a mobile robot equipped with a camera.
- Can identify targets in the field under both light and dark conditions.
- Aims the camera at the detected targets.
- Reports the threat via a message to a mobile phone.
- Alerts the target both vocally and visually.
- Includes a flashlight that simulates an antenna for jamming communication frequencies, enabling the system to disrupt control of the target and neutralize it autonomously.
  
2.Functional specification of the project:

-The system is based on the Raspberry Pi 4 Model B.
-The system includes a Raspberry Pi v2 digital camera, and the system controller processes data received from the camera.
-The system is capable of detecting moving targets using image processing from the camera.
-The system allows the camera to move along two axes using servo motors.
-Image processing is performed in Python using OpenCV algorithms.
-The user interface is written in Python 3.
-The system will send a notification to the user upon detecting a target.
-The system will display the detected target on the screen.






