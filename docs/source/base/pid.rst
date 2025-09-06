PID
===

In POPLib, PID Constants are stored in PIDConfigs that are then applied to a Motor. 
These constants can later be changed by creating a new PIDConfig and sending it to a Motor,
which we will discuss later.

Let us first look at the constructor for creating a PIDConfig object. It is rather simple:

.. code-block:: java

    public PIDConfig(double P, double I, double D, double F)

This is a rather straight-forward constructor, the P, I, and D terms are your standard proportional, 
integral, and derivative gains, and the F term is a velocity feedforward gain. If you plan to use a 
seperate feedforward scheme in your subsystem (such as elevator feedforward), then set F to zero.
If you apply non-zero gains to a PIDConfig object and pass it MotorConfig object, and then create a 
Motor object using that (see :doc:`/base/motors` Documentation), then that Motor will enable all of the 
PID specific methods and commands, which we will discuss a bit later. 

However, if you do not want to use PID for a motor, you can either create a new PIDConfig with all zero 
gains, or use the empty PIDConfig constructor

.. code-block:: java

    PIDConfig configForMotorThatDoesntUsePID = new PIDConfig(0, 0, 0, 0);
    PIDConfig configForMotorThatDoesntUsePID = new PIDConfig();

Both of these will do the same thing. Now, when you pass this into your MotorConfig constructor,
and create your Motor, it will mark this motor as a "PIDless" motor and will print errors to the 
DriverStation if you attempt to use any of the PID specific methods, which we will discuss below.

So how do you use PID on your motors? Assuming you have created a PIDConfig with non-zero gains,
passed it into a MotorConfig, and used the MotorConfig to create a Motor (again see :doc:`/base/motors`), 
then we can access methods that allow us to easily use PID control on our motors.

For these examples, we assume you have created a Motor object like so:

.. code-block:: java
    
    Motor motor = new Motor(Constants.SUBSYSTEM_NAME.MotorConfig);

The first method that we will use allows us to set the desired target position of a motor. It is 
important to note that you should be calling this is in the periodic loop of your subsystem. The 
method looks like this:

.. code-block:: java

    motor.setTargetPosition(position, Constants.ENABLE_FOC, feedforwardOutput);

Let us go through the parameters of this together. It is important to note that only the first parameter 
(the desired position) is required. Also note that, behind the scenes, the gear ratio that you provided 
in ConversionConfig (if you gave one) has already been applied to the motor. Thus, if you want the motor 
to spin three rotations (and you applied the correct gear ratio) it will spin three rotations. If you haven't
applied a gear ratio, that also fine. We recommend using :doc:`/application/tunable numbers` as your "setpoint"
if you want to tune your setpoints. Of course, don't worry about this for now, as you will likely be using the 
application layer of POPLib and POPLib's pre-made subsystems, which already have that fuctionality.

Now lets move on to the other two parameters. Its important to note that these are OPTIONAL parameters. The 
first one is a boolean that describes whether or not to use FOC on this motor. This will only really affect 
Talon devices. FOC allows you to run a motor 15% faster for some trade-offs. However, it requires a Phoenix 
Pro license. This license also needs to be applied to all of the motors using Phoenix Tuner X.

The second is what feedforwardOutput to add to the motor. This is again, OPTIONAL. This would typically be 
calculated by using a WPILIB feedforward object (like ElevatorFeedForward or ArmFeedForward) and calling 
feedforward.calculate. To learn more about feedforward, read the WPILib docs. Note that the feedforwardOutput 
should be in volts (which is the default for WPILIB).

Everything we have talked about so far is for Position PID, but obviously, POPLib (aka the best library on the 
planet) also comes with method for Velocity PID:

.. code-block:: java

    motor.setTargetVelocity(velocity, Constants.ENABLE_FOC, feedforwardOutput);
    
Everything that was previously mentioned about the parameters also applies here.

Of course, you will also want to check if you reached a setpoint if you want to end a command, and POPLib provides
a way to do that too:

.. code-block:: java

    boolean motorIsAtPositionSetpoint = motor.atPositionSetpoint(position, Constants.ACCEPTABLE_PID_TOLERANCE);
    boolean motorIsAtVelocitySetpoint = motor.atVelocitySetpoint(velocity, Constants.ACCEPTABLE_PID_TOLERANCE);

Where the "position" and "velocity" parameters are the setpoints to check (should be the same setpoints that you 
used for setTargetPosition/setTargetVelocity) and the tolerance/error is how close the motor needs to be to the 
setpoint. Note that this uses relative encoders.

PID Tuning
==========

POPLib provides our own PIDTuning class that can be used to tune PID efficiently and easily. The class allows you 
to change your PID variables on the fly, and all you have to do is set it up. All pre-made subsystem from POPLib that  
use PID Control come with the PIDTuning class already set up, all you have to do is put it into tuning mode (this is 
discussed later in the application layer docs). 

Lets go over a overview of how to use the PIDTuning class for a custom subsystem.

.. code-block:: java

    public Subsystem() {
        Motor motor = new Motor(Constants.motorConfig);
        TunableNumber setpoint = new TunableNumber("Motor Setpoing", 0.0, Constants.tuningModeEnabled);
        PIDTuning pidTuning = new PIDTuning("Example Motor", Constants.PIDConfig, Constants.tuningModeEnabled);
    }

This will create a new motor and pidTuning class. The parameters of the PIDTuning class are as follows: the motor 
name that will be used in SmartDashboard (to learn more about SmartDashboard read the WPILIB docs), the PIDConfig 
object that you used to create the motor, and whether or not to turn on tuning mode. We are also using a new concept 
called TunableNumbers, which is a number that can be update from SmartDashboard. For more information, see 
:doc:`/application/tunable numbers`.

.. note:: 

    When you are creating a PIDConfig for tuning mode, make sure to initalize it with non-zero values before passing 
    it in to MotorConfig. If you don't, POPLib will print some error messages in SmartDashboard.

Now lets look at how to use it:

.. code-block:: java

    @Override
    public void periodic() {
        motor.changePID(tuning.generatePIDConfig());
        SmartDashboard.putNumber("motor position", motor.getPosition());
        SmartDashboard.putNumber("motor velocity", motor.getVelocity());
        motor.setTargetPosition(setpoint.get());
    }

This will first change the PID Constants that are applied to the motor, then will log encoder values to SmartDashboard 
for later use when we want to create graphs. Finnally, it runs the PID on the motor using setTargetPosition and 
setpoint.get().

Now, you can go into the Glass tool, click on "SmartDashboard" and then "Tunable Numbers", and then you can change your 
PID Constants and your setpoint value. You can also make graphs in Glass for easy tuning. Have Fun!
