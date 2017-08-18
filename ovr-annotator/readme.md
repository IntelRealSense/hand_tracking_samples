# OVR Annotator - for data collection on Oculus Rift using LibOVR

This unsupported sample program is a VR version of the realtime-annotator.  
This VR annotator has only been tested with the June2017 version of the OVR SDK and only with Visual Studio 2015 on Windows.
Other versions of SDK or Visual Studio may require additional effort to run.

Instructions
------------
Oculus headset and librealsense depth camera (such as SR300) is required.  
To build:
* Be familiar with data collection by trying out `realtime-annotator` before trying this project.
* Go and first build and test `ovr-hand-tracker` before attempting to build/run this one.  Resolve any linker issues as documented.
* Set ovr-annotator to startup project if using visual studio (either use project menu or right-click-menu on the ovr-annotator project)
* Plug in RealSense camera and somehow attach it (tape or string) to the front or top of the headset facing forward.
* Browse the source code in ovrannotator.cpp to get an idea what keyboard and Rift remote inputs do.
* Locate your Oculus Rift Remote (will be easier to use than keyboard while wearing a HMD)
* Run [F5] the program and put on the HMD.
* When tracking the up direction will cycle between fingers free, fingers locked, and fingers with thumb locked.  
more green in the background means more joints are locked.
* When tracking the select button will turn on/off the capture of frames (red tint to background means recording)
* The back controller button toggles inspection mode.   
while in selection mode, frames can be deleted with select while left/right or up/down move through the timeline.
* each time inspection mode is exited, the buffered frames are saved to disk.
* after each capture session use annotation-fixer to verify correctness of any data that was collected

Like the other samples, this only works with a single right hand in the near vicinity of the depth camera.
The point-cloud-depth-mesh is rendered in the same location as the tracking hand model.   
Tried to use appropriate alpha blending to visualize both at once.
Feel free to replace any of this quick-prototyping programmer-art-centric 
reference-code with your own graphics engine and content.

Feedback on the github forum is welcome.
