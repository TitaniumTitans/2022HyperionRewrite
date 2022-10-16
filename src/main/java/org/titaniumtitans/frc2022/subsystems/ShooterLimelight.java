package org.titaniumtitans.frc2022.subsystems;

import edu.wpi.first.math.util.Units;
import org.titaniumtitans.frc2022.Constants.ShooterConstants;
import com.gos.lib.sensors.LimelightSensor;

public class ShooterLimelight {
    private final LimelightSensor m_limeLight;

    public ShooterLimelight(){
        m_limeLight = new LimelightSensor();
    }

    public double getTX(){
        return m_limeLight.getHorizontalAngleDegrees();
    }

    public double getTY(){
        return m_limeLight.getVerticalAngleDegrees();
    }

    public boolean getTV(){
        return m_limeLight.isVisible();
    }

    public double getDistanceFromGoal(double targetHight, double cameraHeight, double cameraAngle){
        return targetHight - cameraHeight / 
        Math.tan(Units.degreesToRadians(getTX() + cameraAngle));
    }

    public double calcRPM(){
        if(!getTV()){
            System.out.println(getTV());
            return 0.0;
        }

        double dist = m_limeLight.getDistance(Units.inchesToMeters(ShooterConstants.kLimelightHeight),
        Units.inchesToMeters(ShooterConstants.kTargetHeight), 
        Units.degreesToRadians(ShooterConstants.kLimelightAngle));
        double rpm = 0.07 * Math.pow((dist - 100), 2) + 3120;
        System.out.println("RPM is:" + rpm);
        return rpm;
    }
    
}
