Absolute Encoders
=================

There are many use cases for absolute encoders, typically as a QOL feature for Swerve. 
Because of this, we provide an easy-to-use API for absolute encoders. Our team only uses 
one type of absolute encoder: a CANCoder. To create a new CANCoder, you first need to create a 
CANCoderConfig:

.. code-block:: java

    CANCoderConfig canCoderConfig = new CANCoderConfig(CAN_ID, CAN_BUS_NAME, OFFSET, INVERTED);

We first require you to give the CAN ID of the CANCoder. Then, we ask you the CAN Bus name 
that the CANCoder is on. We then ask you the offset that should be applied to this CANCoders. 
CANCoders are absolute encoders, which means that they will remember this offset and can be used 
to move a motor back to the same place every time, even when the robot has just started up. This 
differs from normal encoders as those need to be zeroed out at the start of every match to be usable, 
and can't "remember" anything. Finally, we ask if the CANCoder is inverted. We can now use this config 
to create a CANCoder:

.. code-block:: java

    CANCoder encoder = new CANCoder(Constants.canCoderConfig);

.. note:: 

    Whenever a CAN device (like a CANCoder) is created in POPLib, it is registered in the internal 
    CAN ID Registry. If a duplicate CAN ID is found, an error message will be printed out to 
    DriverStation (instead of actually crashing the robot program). Please take POPLib DriverStation 
    error messages seriously. Duplicate CAN IDs make it difficult to debug a robot during competition 
    and can lead to unexcepted errors.

There is only one use of the CANCoder: moving your motors to thier "zero" position at the start of a match. 
Lets say that we already have created a Motor variable (named motor) that is configured with PID. If we 
want to move it to the zero position, we would do something like this:

.. code-block:: java

    motor.setTargetPosition(encoder.getPosition().getRotations());

