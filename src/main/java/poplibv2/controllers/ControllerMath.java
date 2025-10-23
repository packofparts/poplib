package poplibv2.controllers;

import java.lang.Math;

public class ControllerMath {
    /**
     * Returns the cube of a number (number^3)
     * @param triggerVal the number to cube
     * @return the cube
     */
    public static double cube(double triggerVal) {
        return Math.pow(triggerVal, 3);
    }

    /**
     * Applies a deadband/deadzone that minizimes controller drift. Also applies scaling based on the deadzone
     * @param initalVal the raw controller value
     * @param deadband The deadzone to be applied
     * @return the scaled output
     */
    public static double applyDeadband(double initalVal, double deadband) {
        if (initalVal < deadband) {
            return 0;
        } else {
            double scaling = (initalVal < 0 ? -1 : 1) * deadband;
            return (initalVal - scaling) / (1 - deadband);
        }
    }
}