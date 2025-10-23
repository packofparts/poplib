Installation
============

Before using POPLib, you must first install the latest version of WPILib, which can be found `here. <https://github.com/wpilibsuite/allwpilib/releases>`_

Then, you will want to create a new project with the project templete "Command Robot." The language of the project should be Java.

After that, please clone our `repository <https://github.com/packofparts/poplib>`_ and copy the **poplib** folder. You will now want to go back into your project and paste your the poplib folder so that your file structure looks like so.

::

    project_name
    ├── .Glass
    ├── .gradle
    ├── .vscode
    ├── .wpilib
    ├── build
    ├── gradle
    ├── misc
    ├── src/main
    |   ├── deploy
    │   ├── java
    |   |   ├── frc/robot
    |   │   └── poplib
    ├── vendordeps
    ├── .gitignore         
    ├── build.gradle         
    ├── gradlew
    ├── gradlew.bat
    ├── settings.gradle
    └── WPILib-License.md

.. note::
    This is the current installation method. We are also working on publishing POPLib as a github package, but there are several challenges with that.

Now that you are finished installing POPLib, you can check out the POPLib :doc:`overview` to see how POPLib works and how you can use it to speed up robot programming during the competition season.
