// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.commands;

import org.titaniumtitans.frc2022.subsystems.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

//TODO document logic for up/down

public class ClimberManualJoystick extends CommandBase {
  Climber m_climber;
  boolean isUp;
  double speed;
  /** Creates a new ClimberManualJoystick. */
  public ClimberManualJoystick(Climber climber, boolean up) {
    m_climber = climber;
    isUp = up;
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
    if (isUp == true){
      m_climber.joystickControl(speed, isUp);
    }
    if(isUp == false){
      m_climber.joystickControl(-speed, isUp);
    }
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.joystickControl(0.0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
