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
WPILIB's Robot Coordinate System. Next we ask you for the maximum allowed ambiguity 
and distance that an April Tag detection can have for it to still be considered valid. 
The higher these numbers, more "accurate" your April Tag detections will be but you 
will detect less AprilTags. Finally, we ask for the April Tag map that tell us where 
each April Tag is on the field. This is given to you by WPILIB.

We can now use the Config to create a new PhotonVision Camera:

.. code-block:: java

    Camera camera = new Camera(Constants.CAMERA_CONFIG);

We can now use this camera in various ways. One of the most popular ways of using a 
PhotonVision Camera is like so. Let us assume you have created a variable of the type 
SwerveDrivePoseEstimator (which is a WPILIB class, so go read those docs for more info). 
Let us call this variable "poseEstimator". You can then do something like this in your 
drivebase's periodic loop (note that this is done for you if you are using POPLib Swerve):

.. code-block:: java

    Optional<EstimatedRobotPose> estPose = camera.getEstimatedPose(this.getOdomPose());
    if (estPose.isPresent()) {
        this.odom.addVisionMeasurement(estPose.get().estimatedPose.toPose2d(), 
                                       estPose.get().timestampSeconds, 
                                       camera.getVisionStdDevs());
    }

This is how you do vision-based pose estimation with variable standard deviations. That's 
it, if you use the magic of POPLib. 

Now, the second way to use the Camera API is to retieve the distance from the that the 
camera is from the April Tag, which is helpful for vision-based alignment. Do something 
like this:

.. code-block:: java

    Pose2d tagToCameraDifference;
    Optional<Pose2d> possiblePose = camera.relativeDistanceFromCameraToAprilTag();
    if (possiblePose.isPresent()) {
        tagToCameraDifference = possiblePose.get();
    }

