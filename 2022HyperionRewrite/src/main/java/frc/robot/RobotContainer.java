// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.CargoShoot;
import frc.robot.commands.IntakeExtend;
import frc.robot.commands.IntakeRetract;
import frc.robot.commands.ShooterToRPM;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
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

    activateIntake.whenHeld(new IntakeExtend(m_indexer)).whenReleased(new IntakeRetract(m_indexer));
    shooterActivate.whenHeld(new ShooterToRPM(m_shooter, 1500.0));
    cargoShoot.whenActive(new CargoShoot(m_shooter, m_indexer, 1500.0));
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
