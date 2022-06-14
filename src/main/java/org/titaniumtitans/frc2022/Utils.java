package org.titaniumtitans.frc2022;

public class Utils {
    public static double deadBand(double input){
        if(Math.abs(input) < 0.1){
            return 0.0;
        }
        return input;
    }
}
