# Hand Tracking With Realtime Input

This application shows simple real-time hand tracking in action.   
A RealSense depth camera is required to run.  
When running the program place the right hand in the view volume and 
the tracking system will attempt to reconstruct the user's hand pose and 
render a hand model accordingly.   The raw depth data is 
also shown slightly offset from the model

For best results, be sure that there is nothing but the right hand in the camera view.   
It may be helpful to adjust the hand model (+/- keys) to better fit the user's hand.
The hardcoded segmentation routine also assumes an egocentric usage meaning that it needs 
to see where the wrist enters the scene.  Additionally the provided CNN was mostly trained 
from egocentric data collected using a depth camera attached to a VR HMD.


