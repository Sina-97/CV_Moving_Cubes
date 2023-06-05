# Moving cubes with the help of robot : Computer Vision
A computer vision project for moving cubes with 3 different colors with the help of a robot.

In this project, the first step is camera calibration. For that purpose, we need to measure and find some 3D world points from a specific pose of the robot and cubes and take a picture. From the taken picture, we extract the location of corresponding 3D points in 2D coordinations. After that, we use the camera calibration function and find the
projection matrix.

The next step would be finding the distances and angles so that robot be able to collect the cubes and drop them on the circle destination with the same color 
as the cube.
