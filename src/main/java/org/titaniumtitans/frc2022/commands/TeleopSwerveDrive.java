// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.titaniumtitans.frc2022.subsystems.DriveSubsystem;
import org.titaniumtitans.frc2022.Utils;

public class TeleopSwerveDrive extends CommandBase {
    DriveSubsystem m_drive;
    XboxController m_controller;
    /** Creates a new TeleopSwerveDrive. */
    public TeleopSwerveDrive(DriveSubsystem drive, XboxController controller) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_drive = drive;
        m_controller = controller;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xSpeed = Utils.deadBand(m_controller.getLeftX());
        double ySpeed = Utils.deadBand(m_controller.getLeftY());
        double rot = Utils.deadBand(m_controller.getRightX());
        m_drive.drive(xSpeed, ySpeed, rot, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
