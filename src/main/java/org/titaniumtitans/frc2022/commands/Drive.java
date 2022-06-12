package org.titaniumtitans.frc2022.commands;

import org.titaniumtitans.frc2022.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This command drives the {@link SwerveDrive} subsystem.
 */
public class Drive extends CommandBase {
    private final SwerveDrive m_drive;
    private final double xSpeed;
    private final double ySpeed;
    private final double rot;
    private final boolean fieldRelative;

    /**
     * Constructor
     * 
     * @param drive         A {@link SwerveDrive} object representing the swerve
     *                      drive subsystem.
     * @param xspeed        Desired speed in the x direction, in meters per second.
     * @param yspeed        Desired speed in the y direction, in meters per second.
     * @param rotation      Desired robot rotation, measured in radians. (CCW is in
     *                      the positive direction)
     * @param fieldrelative When true, the robot is driven relative to the field,
     *                      instead of relative to its current position.
     */
    public Drive(SwerveDrive drive, double xspeed, double yspeed, double rotation, boolean fieldrelative) {
        m_drive = drive;
        xSpeed = xspeed;
        ySpeed = yspeed;
        rot = rotation;
        fieldRelative = fieldrelative;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_drive.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}