package org.titaniumtitans.frc2022.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.titaniumtitans.frc2022.Constants.ShooterConstants;

public class ShooterLimelight {

    public double getTX(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    public double getTY(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }

    public double getTA(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    }

    public double getDistanceFromGoal(double targetHight, double cameraHeight, double cameraAngle){
        return targetHight - cameraHeight / 
        Math.tan(Units.degreesToRadians(getTX() + cameraAngle));
    }

    public double calcRPM(){
        if(getTA() == 0){
            return 0.0;
        }
        
        double dist = getDistanceFromGoal(ShooterConstants.kTargetHeight, ShooterConstants.kLimelightHeight, ShooterConstants.kLimelightAngle);
        return 0.07 * Math.pow((dist - 100), 2) + 3120;
    }
    
}
