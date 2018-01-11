important:
	process_images.py is the main code for training and classifying
		most of it is function and class definitions
		the actual code is at the bottom
	teach_images folder is where all the training images are stored
		process_images.py needs to access these files to learn
less important:
	rename_files.py is used to rename and move images in bulk
	take_pictures.py allows you to capture and save multiple images with webcam



Currently, the code can discern 5 different shapes:
	Circle, Cross, Square, Triangle 

It is trained to recognize those shapes on images and camera feed:
	- mainly with solid interior color
	- with fairly uniform background colour
	- blurred (out of focus) images
	- rotated and scewed images
	- different lighting conditions and shadows
	- not too good with too much clutter