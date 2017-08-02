# Realtime Hand Annotation Capture Utility

This program is for capturing and saving depth data along with the full hand pose for that frame.  

Motivation
----------
Labeled frames with known pose can be used to train classifiers, such as CNNs, 
to help estimate the hand pose of a new frame with unknown pose.  
Typically a trained CNN can provide information such as whether the palm if facing toward vs away from the camera, 
which depth pixels correspond to which finger, gesture identification, and so on.  
To support any possible training usage,
the labels saved are the full pose (position and orientation) 3D transform information of all 17 bone in the hand (3 per finger, palm, and wrist). 
From this information, all other landmarks (fingertip position, palm center, pointing direction, etc) are trivially derived.
The sample CNN used in these viewers is trained to approximate 2D fingertip locations 
as well as some relative angles indicating finger clenched vs extended state.

Auto Label Capability
---------------------
Collecting ground truth data would be easier with special markers or multiple cameras, 
but it is still possible to do so with only a single depth camera.
Auto generation of the labels (full hand pose) does present a "chicken and egg" problem.  
Even without a CNN or classifier's help, the code here has a tracking system that 
simulates a 3D articulated model of the hand which can incrementally update its pose to 
fit the depth data point cloud each frame.   
The idea is that, rather than trying to label images after the images have been taking,
instead just save out any frames (along with the current hand pose) when this hand tracking system is working.
To label difficult poses, start from a known or reset pose to regain tracking 
and slowly move the fingers in to such other poses.
For auto label data collection, its not necessary to recognize a pose from scratch, only to be able to move to that pose without the tracking getting lost.
To handle occlusions, there is a feature to hold the fingers to a specific pose but still allow the wrist and palm to move and rotate.
So create more challenging hand poses, start with the palm facing the camera to maximize visible finger geometry, 
hold (press [h]) to lock the finger relative angles, then slowly turn and twist the hand as a single unit.
Stop recording if tracking gets lost.
Another option with some RealSense stereo based depth cameras is to use a mirror to get additional backside pointcloud data (support needs to be updated).

Any actual usage of hand tracking will not have a mirror and will have to handle fast movements.  
Once enough data has been collected, and a CNN is adequately trained, 
a tracking system will be able to immediately jump to the correct pose from any initial frame, 
thus providing a more robust hand tracking solution. 


Instructions
-----------
Get used to how to use the program:
* run [F5]  realtime-annotator  (set as startup project if necessary)
* ensure the space in front of the depth camera is clear of any objects
* place right hand in front of the depth camera
* first two seconds it will try to initialize the tracking (using current cnn to help)
* after two seconds the background color changes to a green tint
* system is currently tracking frame to frame
* adjust hand size with +/- keys or edit the model file if necessary (see notes about this topic)
* while tracking move hand and fingers slowly 
* if system is working hit the [r] key to record these frames and associated pose 
* if tracking gets lost, stop recording, remove hand from the view volume and then put it back to re-initialize the tracking.

Best to record in short bursts
* record a short segment
* hit [i] to inspect recently captured frames
* scrub through the timeline with the arrow keys
* hit [bksp] to delete unwanted frames (unless you want to manually edit them later)
* hit key [s] to save frames you want to keep

Data is exported into the folder `../datasets/`.  Each capture session is split into 4 files:
* .rs    the depth data N frames of w*h unsigned short binary data (no file header) 
* .ir    the monochrome ir data N frames of w*h unsigned char binary data (no file header) 
* .pose  the full hand pose N lines ASCII, one line per frame which consists of vector quaternion pair per bone or 17x7 floating point numbers
* .json  camera intrinsics and other capture session information


