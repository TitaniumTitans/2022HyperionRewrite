// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {

  private final SwerveDriveModule m_frontLeft = 
    new SwerveDriveModule(Constants.SwerveConstants.frontLeftDrive, turningPort, encoderPort, encoderReversed)

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
