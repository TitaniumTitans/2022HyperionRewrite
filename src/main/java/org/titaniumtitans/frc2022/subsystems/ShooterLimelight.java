package org.titaniumtitans.frc2022.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterLimelight {

    public double getTX(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    public double getTY(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }

    public double getDistanceFromGoal(double targetHight, double cameraHeight, double cameraAngle){
        return targetHight - cameraHeight / 
        Math.tan(Units.degreesToRadians(getTX() + cameraAngle));
    }
    
}
