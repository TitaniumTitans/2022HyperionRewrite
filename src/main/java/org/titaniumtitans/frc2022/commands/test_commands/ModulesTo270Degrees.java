// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.commands.test_commands;

import org.titaniumtitans.frc2022.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ModulesTo270Degrees extends CommandBase {
  DriveSubsystem m_drive;
  /** Creates a new ModulesTo90Degrees. */
  public ModulesTo270Degrees(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(0, -1, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
