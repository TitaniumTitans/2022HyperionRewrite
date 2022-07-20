// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import org.titaniumtitans.frc2022.commands.CargoShoot;
import org.titaniumtitans.frc2022.commands.IntakeExtend;
import org.titaniumtitans.frc2022.commands.IntakeRetract;
import org.titaniumtitans.frc2022.commands.ShooterToRPM;
import org.titaniumtitans.frc2022.commands.TeleopSwerveDrive;
import org.titaniumtitans.frc2022.commands.test_commands.ModulesTo180Degrees;
import org.titaniumtitans.frc2022.commands.test_commands.ModulesTo270Degrees;
import org.titaniumtitans.frc2022.commands.test_commands.ModulesTo360Degrees;
import org.titaniumtitans.frc2022.commands.test_commands.ModulesTo90Degrees;
import org.titaniumtitans.frc2022.Constants.OIConstants;
import org.titaniumtitans.frc2022.subsystems.DriveSubsystem;
import org.titaniumtitans.frc2022.subsystems.Indexer;
import org.titaniumtitans.frc2022.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/*
 * This class is where the bulk of the robot should be declared.    Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).    Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Indexer m_indexer = new Indexer();
    private final Shooter m_shooter = new Shooter();

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        ShuffleboardTab testCommands = Shuffleboard.getTab("Test Commands");

        testCommands.add("Modules to 360", new ModulesTo360Degrees(m_robotDrive));
        testCommands.add("Modules to 270", new ModulesTo270Degrees(m_robotDrive));
        testCommands.add("Modules to 180", new ModulesTo180Degrees(m_robotDrive));
        testCommands.add("Modules to 90", new ModulesTo90Degrees(m_robotDrive));

        // Configure default commands
    
        m_robotDrive.setDefaultCommand(new TeleopSwerveDrive(m_robotDrive, m_driverController));
        
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        JoystickButton activateIntake = new JoystickButton(m_driverController, XboxController.Button.kX.value);
        JoystickButton shooterActivate = new JoystickButton(m_driverController, XboxController.Button.kY.value);

        activateIntake.and(shooterActivate.negate()).whenActive(new IntakeExtend(m_indexer)).whenInactive(new IntakeRetract(m_indexer));
        shooterActivate.and(activateIntake.negate()).whenActive(new ShooterToRPM(m_shooter, 1500)).whenInactive(new ShooterToRPM(m_shooter, 0));
        shooterActivate.and(activateIntake).whenActive(new CargoShoot(m_shooter, m_indexer, 1500));
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
        /*
         * // Create config for trajectory
         * TrajectoryConfig config =
         * new TrajectoryConfig(
         * AutoConstants.kMaxSpeedMetersPerSecond,
         * AutoConstants.kMaxAccelerationMetersPerSecondSquared)
         * // Add kinematics to ensure max speed is actually obeyed
         * .setKinematics(DriveConstants.kDriveKinematics);
         * 
         * // An example trajectory to follow. All units in meters.
         * Trajectory exampleTrajectory =
         * TrajectoryGenerator.generateTrajectory(
         * // Start at the origin facing the +X direction
         * new Pose2d(0, 0, new Rotation2d(0)),
         * // Pass through these two interior waypoints, making an 's' curve path
         * List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
         * // End 3 meters straight ahead of where we started, facing forward
         * new Pose2d(3, 0, new Rotation2d(0)),
         * config);
         * 
         * var thetaController =
         * new ProfiledPIDController(
         * AutoConstants.kPThetaController, 0, 0,
         * AutoConstants.kThetaControllerConstraints);
         * thetaController.enableContinuousInput(-Math.PI, Math.PI);
         * 
         * SwerveControllerCommand swerveControllerCommand =
         * new SwerveControllerCommand(
         * exampleTrajectory,
         * m_robotDrive::getPose, // Functional interface to feed supplier
         * DriveConstants.kDriveKinematics,
         * 
         * // Position controllers
         * new PIDController(AutoConstants.kPXController, 0, 0),
         * new PIDController(AutoConstants.kPYController, 0, 0),
         * thetaController,
         * m_robotDrive::setModuleStates,
         * m_robotDrive);
         * 
         * // Reset odometry to the starting pose of the trajectory.
         * m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
         * 
         * // Run path following command, then stop at the end.
         * return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
         * false));
         */
    }
}
