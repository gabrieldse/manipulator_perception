
"""
Ref 2 : https://www.traimaocv.fr/CoursStereoVision/co/ModCamera_1.html

La taille du pixel est :

"« CMOS Sensor Data »"

"« The CMOS sensor in the Logitech C270 webcam features the following data : »"

"« Sensor Resolution = 1280 x 960 »"

"« Pixel Dimension = 2,8 μm x 2,8 μm »"

"« Sensor Dimension = 3,5 mm x 2,7 mm »"

"« Sensor area = 9,45 mm2 »"
"""

"""
Camera parameters
focal distance = 4.0 mm
focal lenght = 1350px
FOV - 60°
Optical Resolution (True)	1280 x 960 1.2MP
Image Capture (4:3 SD)	320x240, 640x480 1.2 MP, 3.0 MP
Image Capture (16:9 W)	360p, 480p, 720p
Video Capture (4:3 SD)	320x240, 640x480, 800x600
Video Capture (16:9 W)	360p, 480p, 720p,
Frame Rate (max)	30fps @ 640x480

1430 px

"""


**** Calibrating ****
mono pinhole calibration...
D = [-0.20138377333160384, 0.12392068612484416, -0.013412206972079874, -0.0017584552066244466, 0.0]
K = [595.3144843089073, 0.0, 345.021173655603, 0.0, 604.8528227241521, 165.44422990553628, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [564.4595947265625, 0.0, 345.55359683960705, 0.0, 0.0, 572.9318237304688, 157.26571186404726, 0.0, 0.0, 0.0, 1.0, 0.0]
None
# oST version 5.0 parameters


[image]

width
640

height
480

[narrow_stereo]

camera matrix
595.314484 0.000000 345.021174
0.000000 604.852823 165.444230
0.000000 0.000000 1.000000

distortion
-0.201384 0.123921 -0.013412 -0.001758 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
564.459595 0.000000 345.553597 0.000000
0.000000 572.931824 157.265712 0.000000
0.000000 0.000000 1.000000 0.000000