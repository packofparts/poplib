Overview
========

POPLib can essientally be split up into two parts: the "base" layer and the "application" layer. 
The "base" layer includes hardware abstractions to make robot programming for these things easier. This includes a motor class that handles the brunt of the PID work, a PhotonVision Camera class that helps in retrieving useful infomation from the Photonvision Camera without having a lot of boilerplate code and complex math, a Gyro class, an Absolute Encoder class ... you get the point. If you just want to use the base layer of POPLib ... that's great! However, there is also the "application" layer of POPLib.

The "application" layer of POPLib directly uses the "base" layer to create pre-made, ready to go subsystems. These subsystems include a swerve drivebase, an elevator implementation, a flywheel subystem, a pnuematics subsystem, etc. Since these subsystems are "ready to go" there is very little code that you have to write. All you need to do is fill out some constants (Ok, a lot of constants) so that the subsystems know what type of hardware thier dealing with, what PID Constants to use (if needed), etc.

Regardless, it's important to first familiarize yourself with the :doc:`base` layer of POPLib, as using the "application" layer requires some knowledge of how to use the base layer. 

After reading the base layer documentation, you can read the :doc:`application` layer documentation that details how to use POPLib's premade subsystems.