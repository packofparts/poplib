Photonvision Cameras
====================

Here at Pack of Parts, we use our PhotonVision Camera to detect April Tags and 
do vision-based pose estimation, and we use our limelight cameras to detect objects 
on the ground (such as gamepieces). This page will be dedicated to covering the 
Camera API, which represents PHOTONVISION Cameras.

To create a new Camera, we of course have to create a new CameraConfig. Lets take a 
look at the constructor:

.. code-block:: java

    public CameraConfig(String cameraName, 
                        Transform3d cameraToRobot, 
                        double poseAmbiguityThreshold, 
                        double poseDistanceThreshold, 
                        StdDevStategy stdDevStategy, 
                        AprilTagFields thisYearsField)

The first parameter is the name of the Camera. Note that this is the name of the 
CAMERA as in the PhotonVision settings and NOT the name of the computer that is 
running PhotonVision. The second parameter is the position of the camera's focal 
lens relative to the center of the robot, represented as a Transform3d and in 
WPILIB's Robot Coordinate System.