Motors
======

In POPLib, Motors are represented using the Motor.java API. 
The main reason to use the Motor object to represent your motors is that the Motor object abstracts away 
many of the differences between vendor specific APIs (e.g. the TalonFX and SparkMax APIs). 
This allows you to quickly change what type of motor you are using in code 
(e.g. Using the Motor.java API, I can easily change between a SparkMax motor and a TalonFX motor). 
This also allows the application layer of POPLib to work with any type of motor that may be on your robot, 
without you changing your code.

So, how do you create a Motor? In POPLib, we use Config objects as containers 
and helper objects that assist in the creation of hardware objects. To that end, 
we must first create a MotorConfig object before creating a Motor object. 
Lets take a look at how to do that by looking at the constructor:

.. code-block:: java

    public MotorConfig(int canID,
                       String canBUS,
                       MotorVendor motorVendor,
                       PIDConfig pidConfig,
                       int currentLimit,
                       boolean inverted,
                       IdleBehavior idleMode,
                       ConversionConfig conversionConfig)

Lets go through each parameter. The canID is pretty simple: its the CAN ID of the motor controller 
(the Spark/Kraken on the robot). This can be set using vendor specific tools 
(Phoenix Tuner X for CTRE Devices and REV Hardware Client for Sparks). The canBUS parameter 
allows you to specific what CAN Bus this motor is on (typically only used if you are using a CANnivore). 
The default canBUS should be "rio".

.. note::

    Currently, if you specify a CAN Bus for Spark devices, it won't apply. 
    This is because the Vendor provided SparkMax API has no way to specify what CAN Bus a motor is on. 
    Thus, all Sparks should be put on the RoboRio CAN Bus loop. 
    However, the CAN Bus parameter still works for TalonFX devices.

Then, you need to specify what type of motor this is using the MotorVendor enum. 
This is how the Motor API knows which native vendor API to use. 
After this, you need to create a PIDConfig. For more information on PID and PIDConfigs, 
see :doc:`pid`. You then need to specify the current limit for this motor. 
The recommended current limits for your motor can be found using the vendor documentation for your motor. 
After this, specify if the motor signals need to be inverted. Then, specify the IdleBehavior for this motor.
This is the default behavior of the motor when it is not receiving active commands to run. 
The IdleBehavior can be specified using the IdleBehavior enum, and the options are Brake and Coast. 
Brake mode will have the motor actively resist external forces, and coast will allow the motor to spin freely. 
Finally, you must pass in a ConversionConfig object. This object details the gear ratio between the motor shaft 
rotation and the final mechanism rotation. If you don't want to deal with gear ratios and conversions, 
simply create a new ConversionConfig object without any parameters. However, if you do want the 
Motor API to automatically consider gear ratios, create a new ConversionConfig object using this constructor:

.. code-block:: java

    public ConversionConfig(double gearRatio, AngleUnit units)

Now that you have created a MotorConfig object, we can create a new Motor like this:

.. code-block:: java

    Motor motor = new Motor(Constants.SUBSYSTEM_NAME.motorConfig);

.. note::

    Because of how long the MotorConfig constructor is rather long, we recommend creating MotorConfig 
    objects in a file named Constants.java. These MotorConfigs can also be divided by class into what 
    subsystem you are using them for, just as in the above code example.

.. note::

    Whenever a CAN device (like a Motor) is created in POPLib, it is registered in the internal 
    CAN ID Registry. If a duplicate CAN ID is found, an error message will be printed out to 
    DriverStation (instead of actually crashing the robot program). Please take POPLib DriverStation 
    error messages seriously. Duplicate CAN IDs make it difficult to debug a robot during competition 
    and can lead to unexcepted errors.

Now that we have created our Motor, let us discuss how to use it. It is important to note
that any methods/commands that use PID will be discussed in the :doc:`pid` Documentation.

Let us first see how we can set the speed of a motor without using PID:

.. code-block:: java

    motor.set(speed);

Where speed is some number from -1.0 to 1.0.

.. note::

    If you have this motor configured with PID (as in you passed non-zero PID Constants in PIDConfig), 
    then POPLib will print an error message in DriverStation. It is not recommended to mix PID methods 
    with non-PID methods as they can send conflicting commands to the motor.

Now let us look at how to access encoder readings from a motor. Note that this is NOT from an absolutee 
encoder like a CANCoder. See the :doc:`absolute encoders` Documentation for that. To access position and velocity readings 
from your encoder, do something like this:

.. code-block:: java

    double motorPos = motor.getPosition();
    double motorVel = motor.getVelocity();

We can also change the IdleBehavior of a motor after it has been created. Do something like this:

.. code-block:: java

    motor.changeIdleBehavior(IdleBehavior.BRAKE);
    motor.changeIdleBehavior(IdleBehavior.COAST);

The final important thing you need to know how to do is add a follower motor. 
This is extremely useful if you need two motors to run in the exact same direction. 
Lets say you have already created a lead motor (we will use the same motor as we have been previously 
using in our examples). Now lets say that I want a different motor (which I have not created a MotorConfig 
object or Motor object for) to follow my lead motor. I would do something like this:

.. code-block:: java

    motor.addFollower(Constants.SUBSYSTEM_NAME.followerCanId, Constants.SUBSYSTEM_NAME.followerShouldBeInverted);

This will create a new motor behind the scenes and make it follow the commands of the lead motor 
(the variable named "motor" in our examples. We need to pass in the CAN ID of the follower motor 
and whether or not the follower motor should be inverted).

Now that you have read this doc page, please read the docs for :doc:`pid`.