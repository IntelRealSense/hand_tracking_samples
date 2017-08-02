placeholder directory for saving recorded data files of skeletal hand pose and corresponding depth, ir, and color video streams

this is the default directory which is easily changed.

the .rs files contain all the uncompressed depth images of a sequence.
the .ir files contain the uncompressed ir images from a sequence. 
depth pixels are unsigned short (16 bit), ir is 8 bit monochrome.
there is no header on these files and no padding between frames.
dimensions and camera intrinsics are in the .json files.
the pose data of the 17 hand bones is in the .pose files.
there is wrist, palm and 3 bones per finger and thumb for a total of 17.
The reason there are 17 bones instead of 22 that you might expect on a Maya rig is that the 
fingertips (terminal bones in Maya) are redundant and really on there to visualize the 
orientation of the actual fingertip bones (2nd last bone).  
each line of ascii in the .pose file is the position and orientation of each bone represented as vector quat pairs. 
