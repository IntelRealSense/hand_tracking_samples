# CNN Training Utility

It takes a lot of frames to train a CNN.  Typically this will require data from a large number of datasets.  
You may need to compile for 64 bit (x64) if your getting up to the 100000s of frames.
```
cd ../datasets/
../build/train-cnn-pose-net-vs2015_Release_x64.exe *.rs
```

To reduce memory usage the program segments the data on load 
to only keep the 64x64 focused hand segment for each frame.
If there is a lot of data, its possible to inspect the frames while loading continues in a 2nd thread.  
When the program starts up, it will load the default cnn file ```../assets/handposedd.cnnb```  
To start from scratch with a new CNN, you can hit ctrl-x or the corresponding button widget to 
randomly reinitialize the weights.

 
Hit [t], or the training switch widget, to train and let your computer run for a long time (overnight).  
Be sure to hit [CTRL-t] to save the cnn before exiting the program (note to self, implement an autosave).  
It saves the the current directory, which is probably not the default location when any of these programs load their cnn from the parent directory.
The backprop implementation has some SSE SIMD smarts - but doesn't exploit muti-core or gpu.
The architecture of the CNN is pretty typical, similar to Torch's housenumbers sample, but much wider due to the large output vector (heatmaps).

Rather than looking at MSE (mean square error), after training if the output provides crisp heatmap 
locations similar to the labels, then this indicates good learning.   
If you know that a particular geometric feature is a particular finger, 
then the model to point cloud fitting will fine tune the result - the hand will snap 
into the correct pose like a puzzle piece that has been lined up correctly.

For a CNN to be effective in practice requires coverage in the training set.  
So lots and lots of captured data will help.

