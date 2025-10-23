Gyro
====

It is incredibly easy to create a new Gyro object using POPLib. It is first important to 
note that the Gyro class is an abstract class, meaning that you will actally want to create 
a new Pigeon object instead of a new Gryo object (where the Pigeon is a specific type of gyro 
our team uses). 

To create a new Pigeon object simply do this:

.. code-block:: java
    
    PigeonConfig config = new PigeonConfig(Constants.GYRO_CAN_ID, Constants.GYRO_INVERTED, Constants.CAN_BUS_NAME)
    Pigeon gyro = new Pigeon(config);

The constructor is rather simple: it requires the CAN ID of the Pigeon, whether or not the Pigeon 
is inverted (the front of your Pigeon should always be pointing to the front of your robot, and your 
Pigeon should always be at the EXACT center of your robot. If the front of the Pigeon is facing the 
back of your robot, then set this Pigeon as being inverted). Finally, we ask for the name of the CAN 
Bus Loop where this device is located.

.. note::

    Whenever a CAN device (like a Pigeon) is created in POPLib, it is registered in the internal 
    CAN ID Registry. If a duplicate CAN ID is found, an error message will be printed out to 
    DriverStation (instead of actually crashing the robot program). Please take POPLib DriverStation 
    error messages seriously. Duplicate CAN IDs make it difficult to debug a robot during competition 
    and can lead to unexcepted errors.

Now lets move on to how to use the gyro. During the start of your match you will likely want to zero out 
your gyro, which can be done like so:

.. code-block:: java

    gyro.zeroGyro();                       // both of these do the same thing
    gyro.setAngle(new Rotation2d(0));      // both of these do the same thing

One of the main uses for a gyro is to get the yaw angle of your robot, normalized (fit from 0 to 360). This 
may be done like so: 

.. code-block:: java

    Angle rotationOfRobotAsAngle = gyro.getNormalizedAngle();
    Rotation2d rotationOfRobotAsRot2d = gyro.getNormalizedRotation2dAngle();

Again, both of these do the exact same thing, just in different formats. Now, sometimes, you may want to be 
really fancy, and you want to know the angle of the robot with the latency already compensated (because 
technically when you call gyro.getNormalizedAngle(), the value it returns is a few milliseconds old due to 
fact that the electrical signal has to travel from the gyro to the robot brain). If you want to know the 
exact angle of the robot at that exact moment in time use:

.. code-block:: java

    Angle rotationOfRobotWithLatComp = gyro.getLatencyCompensatedAngle();

However, note that this is truly unnessesary and very over the top. Something that may be more helpful is 
knowing the AngluarVelocity of your robot. To get this from your gyro, simply do:

.. code-block:: java

    AngluarVelocity gyroRotVelo = gyro.getAngularVelo();

Finally, there may be cases where you need to access the raw values of the pitch, yaw, and roll of the robot. 
These values will be NOT BE INVERTED and will NOT BE NORMALIZED. They can be accessed like so:

.. code-block:: java

    Angle gyroYaw = gyro.getYaw();
    Angle gyroPitch = gyro.getPitch();
    Angle gyroRoll = gyro.getRoll();


