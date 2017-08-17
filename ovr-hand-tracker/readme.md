# Hand Tracking on Oculus Rift using LibOVR

This unsupported sample program illustrates how a hand tracking system might work at runtime on a VR setup.
This sample has only been tested with the June2017 version of the OVR SDK and only with Visual Studio 2015 on Windows.
Other versions of SDK or Visual Studio may require additional effort to run.

Instructions
------------
Oculus headset and Librealsense depth camera (eg SR300) is required.  
To build:
* download and place a copy of the Oculus SDK in the ./third_party subfolder.  
* You may need to rebuild LibOVR.lib to match c++ compilation settings for runtime library.  see comments in dx_ovr.h for additional details.
* Compile [F7 or Ctl+Shift+B] the project (using your preferred toolset)
* Set ovr-hand-tracker to startup project if using visual studio (either use project menu or right-click-menu on the ovr-hand-tracker project)
* Plug in RealSense camera and somehow attach it (tape or string) to the front or top of the headset facing forward.
* Run [F5] the program and put on the HMD.
* Hitting +/- keys will adjust hand size, 1-4 change hand visualization, and m toggles the offset depth mesh view.

Like the other samples, this only works with a single right hand in the near vicinity of the depth camera.

Feedback on the github forum is welcome.
