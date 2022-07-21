// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.commands;

import org.titaniumtitans.frc2022.subsystems.Climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberManualJoystick extends CommandBase {
  Climber m_climber;
  XboxController m_driverController;
  double speed;
  /** Creates a new ClimberManualJoystick. */
  public ClimberManualJoystick(Climber climber, XboxController driverController) {
    m_climber = climber;
    m_driverController = driverController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = SmartDashboard.getNumber("Climber Speed", 0.0);
    if (m_driverController.getRightBumperPressed()){
      m_climber.joystickControl(speed);
    }
    if(m_driverController.getLeftBumperPressed()){
      m_climber.joystickControl(-speed);
    }
    else{
      m_climber.joystickControl(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.joystickControl(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
