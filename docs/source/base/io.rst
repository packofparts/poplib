Input/Controllers
=================

POPLib also makes it extremely easy to access inputs from controllers and map controller 
buttons to Commands. Lets take a look at how to use POPLib's IO APIs.

There are two controller setups that are programmed into POPLib: Having a setup with two 
Microsoft Flight Simulator Joysticks as drive controllers and one Xbox Controller as an 
operator controller. The second setup, which is what we recommend, is having two Xbox 
controllers, one for the driver and one the operator. Lets take a look at how to create 
objects to represent both of these setups:

.. code-block:: java

    Joysticks input = Joysticks.getInstance();
    // OR
    XboxIO input = XboxIO.getInstance();

Note that, for once, we don't have to pass in a Config object or some parameters. However, 
you must ensure that your physical joysticks/controllers are in the right port in DriverStation. 
For a Joysticks setup, your Microsoft Flight Simulator Joysticks must be on ports 0 and 1, 
while your Xbox controller must be on port 2. For a XboxIO setup, your driver Xbox controller 
must be on port 0 while your operator Xbox controller must be on port 1.

We will now go over how to use the Controller API. Note that no matter your selection of IO 
(Joysticks or XboxIO), the way that these methods/functions work will not change. It was 
designed such that you will easily be able to change what hardware you are using to control 
everything.

Lets first explore how to get access to instructions from the driver on how to move the robot:

.. code-block:: java

    double horizontalMovement = input.getDriveTrainTranslationX();
    double verticalMovement = input.getDriveTrainTranslationY();
    double rotationMovement = input.getDriveTrainRotation();

It is also recommended to apply a deadzone to these values to avoid controller drift:

.. code-block:: java

    horizontalMovement = ControllerMath.applyDeadband(horizontalMovement, Constants.CONTROLLER_DEADZONE);
    verticalMovement = ControllerMath.applyDeadband(verticalMovement, Constants.CONTROLLER_DEADZONE);
    rotationMovement = ControllerMath.applyDeadband(rotationMovement, Constants.CONTROLLER_DEADZONE);

A good value to set controller deadzone to is 0.1 or lower. We can now move on to how to map 
buttons to commands. To do this, we use WPILIB's Triggers. Do something like this:

.. code-block:: java

    input.getDriverButton(XboxController.Button.kA.value).onTrue(exampleCommand);

What this will do is run the "exampleCommand" when the "A" button is pressed on the driver 
controller. We can also do something like this:

.. code-block:: java

    input.getDriverTrigger(XboxController.Axis.kRightTrigger.value).onTrue(exampleCommand);

When the right trigger on the Xbox controller is at least half way pressed, then the program 
will run the exampleCommand. Of course, similar methods are provided for the operator controller 
as well.

.. note:: 

    You do NOT need to call .schedule on your commands using this method. WPILIB will autoschedule 
    them when nessesary.