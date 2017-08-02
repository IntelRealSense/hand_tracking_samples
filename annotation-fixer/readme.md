# Annotation Fixer Utility

Data set inspecting and editor for hand pose and example CNN training.  
Based on depth camera input and 3D articulated model pose information.

Editing a dataset
-----------------
Run the command with one command line argument to inspect and fix up pose annotations for that fileset.  
Use the filename with the .rs at the end as follows:
```
annotation-fixer-vs2015_Release_x64.exe  ../datasets/hand_data_0.rs
```
This will also load the other files (.ir,.pose,.json) corresponding to this dataset.  

The user can tweak individual frames so the model matches how the user's hand in front of the camera was configured.
Often this is easy to do by using the mouse to nudge bones (out of some bad local minimum) and let it snap 
into place by fitting it to the point cloud.
The GUI is minimal, but hopefully it wont be needed much. 
One thing that can happen is that the user's hand was moving too fast during a capture session 
and the tracking didn't keep up.
In this case, just letting the simulation run a bit longer on such frames. 
Rather than spending too much time fixing individual frames,
sometimes its easier to just delete all the bad frames and conduct more capture sessions 
in order to get the required large volumes of training data.

See the source code for more information.



