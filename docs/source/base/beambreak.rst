BeamBreak
=========

In FRC, it is very useful to know if an object is in a mechanism on your robot. To do 
that, a beambreak sensor is typically used, so POPLib provides a helpful BeamBreak API 
to help you query your beambreak. We start by creating a object like so:

.. code-block:: java

    BeamBreak beambreakSensor = new BeamBreak(Constants.BEAMBREAK_PORT_ID);

To create a BeamBreak object, all you need is the port id that the beambreak is on.

.. note:: 

    The "Port Id" is NOT a CAN ID as the beambreak is not on the CAN Loop. It instead 
    directly connects to the roborio's digitial input ports. We are asking what digital 
    input port your beambreak is on. If you dont know, consult your local electrical subteam.

Lets talk about how to use the BeamBreak API. If you want to see if the beambreak sensor is 
blocked, do something like this:

.. code-block:: java

    boolean isObjectBlockingSensor = beambreakSensor.isBlocked();

Another cool thing you can do with the BeamBreak API is command chaining. Say you have a Command 
(we are representing our example command as a variable named "command") and you want to run this 
command until the beambreak is blocked. This would be helpful, say, if you have a beambreak on your 
intake and you want to stop your intake motors once the beambreak detects that a gamepiece is in the 
intake:

.. code-block:: java

    command.until(beambreakSensor.getBlockedSupplier()).andThen(...);

And thats all you really need to know about the BeamBreak API. If you want to know more, read the java 
docs.