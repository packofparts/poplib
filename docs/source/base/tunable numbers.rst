Tunable Numbers
===============

Have you ever wanted to change a constant/number in your code WITHOUT having to 
redeploy your code? Well, look no further. Here at POPLib Inc. we have come up 
with a solution: Tunable Numbers.

Using the TunableNumber API, you can edit a Constant in your code without having 
to redeploy your robot. This is useful for tuning setpoints, and it is also utilized 
by POPLib's PIDTuning API. Here's how you create a new TunableNumber:

.. code-block:: java

    TunableNumber setpoint = new TunableNumber("Subsystem Setpoint", 0.0, Constants.TUNING_MODE);

The first variable is the name of this TunableNumber - which you will need when you 
want to edit the value of the variable later. Then, you need to set a default value 
(I used 0.0 as this is a setpoint). The default value is used when the user has not 
given us a new value to use. Tuning mode dictates whether or not the TunableNumber 
should accept new inputs from a user. Typically, you should turn tuning mode off 
during a competition.

Now lets look at how to get the latest reading from a TunableNumber:

.. code-block:: java

    setpoint.get()     // wow that was really hard and complex

We should also see how to change the value of a TunableNumber in code:

.. code-block:: java

    setpoint.set(someNumber);   // wow, this was also really hard + complex

Now lets get to the part you have been waiting for: changing the value of this 
number WITHOUT having to restart your robot code. To do this, make sure tuning 
mode is on, and open your favorite SmartDashboard viewing tool (Glass or Elastic). 
Then go into SmartDashboard -> TunableNumbers. You should see the name of your 
tunable number there. If you change that number (and dont press enter or space, 
that would disable the robot), then the change will be reflected in your robot code 
(the next time you call setpoint.get(), it will return a different number!).