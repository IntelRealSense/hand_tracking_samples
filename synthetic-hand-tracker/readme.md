# Hand Tracking via Synthetic Depth Data

This sample program illustrates how a hand tracking system might work at runtime.

Instructions
------------
No depth camera is required.  
To run:
* compile [F7 or Ctl+Shift+B] the project (using your preferred toolset)
* set synthetic-hand-tracker to startup project if using visual studio (either use project menu or right-click-menu on the synthetic-hand-tracker project)
* run [F5] the program
* an opengl window should pop up showing the tracking
* dragging using the left mouse button will rotate the depth mesh view and reconstructed hand pose 3D view
* using the [c] key will enable/disable the CNN
* using the [a/f] keys will control the animation speed of the synthetic hand

Explanation
-----------
The flow of the program goes from top to bottom. 

A dummy hand model is animated and rendered in the upper left.
The depth buffer is read back from the gpu, and is
shown as grayscale in the top row to the right of the initial dummy hand rendering.
This depth is our pretend depth camera input.
This depth is rendered as a mesh, or triangulated point cloud, in the left part of the 2nd row.  

This depth is the input to the hand tracking system.  
The system does not know any pose information about 
the animated dummy hand model or where that data came from.
First, the hand tracking system focuses in on the hand section.
For this it uses an over-simplified hand coded segmentation routine 
that assumes the right hand is the only thing in the near view volume.

Next, the segmented and resampled 64x64 image is used as input to the hand tracker's CNN.
The CNN forward propagation is done in a 2nd thread so it isn't necessarily 
producing results every frame. 
The CNN outputs 8 16x16 2D heatmaps for key landmarks and 16 1x16 1D heatmaps for key joint angles.
This is shown in the first tall column to the right of the other views.
To show how well the CNN is performing, beside this in the rightmost column, 
the program also shows heatmaps corresponding to the current pose of the dummy model that was used for rendering.

The CNN outputs and the depth data are used by the hand tracking system to estimate the pose from the input.
The results of this hand tracking are shown in the bottom 3D view.

See source code for more details.
