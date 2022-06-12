// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.titaniumtitans.frc2022.commands.CargoShoot;
import org.titaniumtitans.frc2022.commands.Drive;
import org.titaniumtitans.frc2022.commands.IntakeExtend;
import org.titaniumtitans.frc2022.commands.IntakeRetract;
import org.titaniumtitans.frc2022.commands.ShooterToRPM;
import org.titaniumtitans.frc2022.subsystems.Indexer;
import org.titaniumtitans.frc2022.subsystems.Shooter;
import org.titaniumtitans.frc2022.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Indexer m_indexer = new Indexer();
  private final Shooter m_shooter = new Shooter();
  private final SwerveDrive m_drive = new SwerveDrive();

  //Creates Xbox controller on port 0
  private XboxController m_controller = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //m_indexer.setDefaultCommand(new IntakeRetract(m_indexer));


    JoystickButton activateIntake = new JoystickButton(m_controller, XboxController.Button.kX.value);
    JoystickButton toggleHighGoal = new JoystickButton(m_controller, XboxController.Button.kA.value);
    JoystickButton shooterActivate = new JoystickButton(m_controller, XboxController.Button.kY.value);
    Trigger cargoShoot = new JoystickButton(m_controller, XboxController.Button.kX.value).and(new JoystickButton(m_controller, XboxController.Button.kY.value));

    cargoShoot.whenActive(new CargoShoot(m_shooter, m_indexer, 1000.0)).whenInactive(new ShooterToRPM(m_shooter, 0.0));
    activateIntake.whenHeld(new IntakeExtend(m_indexer)).whenReleased(new IntakeRetract(m_indexer));
    shooterActivate.whenHeld(new ShooterToRPM(m_shooter, 1000.0)).whenReleased(new ShooterToRPM(m_shooter, 0.0));

    m_drive.setDefaultCommand(new Drive(m_drive, m_controller.getLeftX(), m_controller.getLeftY(), m_controller.getRightX(), false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
