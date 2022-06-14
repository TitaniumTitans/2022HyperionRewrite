package org.titaniumtitans.lib;

/**
 * A collection of static utility methods.
 */
public class Utils {
    /**
     * Joystick deadband - The absolute value of the joystick must be larger than a
     * constant for an output to be returned.
     * 
     * @param input The current joystick input value
     * @return The output
     */
    public static double deadBand(double input) {
        if (Math.abs(input) < 0.1) {
            return 0.0;
        }
        return input;
    }
}
