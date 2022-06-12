package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class Drive extends CommandBase{
    private final SwerveDrive m_drive;
    private final double xSpeed;
    private final double ySpeed;
    private final double rot;
    private final boolean fieldRelative;

    public Drive(SwerveDrive drive, double xspeed, double yspeed, double rotation, boolean fieldrelative){
        m_drive = drive;
        xSpeed = xspeed;
        ySpeed = yspeed;
        rot = rotation;
        fieldRelative = fieldrelative;
        addRequirements(drive);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        m_drive.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }

}