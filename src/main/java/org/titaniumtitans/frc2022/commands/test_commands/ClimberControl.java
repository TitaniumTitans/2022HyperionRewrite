// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.commands.test_commands;

import org.titaniumtitans.frc2022.commands.ClimberManualJoystick;
import org.titaniumtitans.frc2022.commands.ClimberPIDControl;
import org.titaniumtitans.frc2022.subsystems.Climber;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberControl extends CommandBase {
  Climber m_climber;
  XboxController m_dController;
  boolean controlMode;

  /** Creates a new ClimberControl. */
  public ClimberControl(Climber m_climber, XboxController m_dController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_climber = m_climber;
    this.m_dController =  m_dController;
    controlMode = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controlMode = SmartDashboard.getBoolean("Joystick Climbers?", false);
    if(controlMode == true){

        double speed = SmartDashboard.getNumber("Climber Speed", 0.0);

        if (m_dController.getRightBumper() == true){
          m_climber.joystickControl(speed, true);
        }

        if(m_dController.getLeftBumper() == true){
          m_climber.joystickControl(-speed, false);
        }
    }
    if(controlMode == false){

       if(m_dController.getRightBumper() == true){
            m_climber.climberPidControl(m_climber.m_leftMaxHeight.getValue(), m_climber.m_rightMaxHeight.getValue());
        }

        if(m_dController.getLeftBumper() == true){
            m_climber.climberPidControl(500, 500);
        }
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
