// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.titaniumtitans.frc2022.Constants.ModuleConstants;
import org.titaniumtitans.lib.Swerve.CTREModuleState;
import org.titaniumtitans.lib.Utils;

public class SwerveModule {

    private static final double TURNING_GEAR_RATION = 21.428;
    private static final double DRIVE_GEAR_RATION = 8.17;

    private final WPI_TalonFX m_driveMotor;
    private final WPI_TalonFX m_turningMotor;

    private final WPI_CANCoder m_turningEncoder;

    private final String m_name;

    private SwerveModuleState m_desired_state;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ModuleConstants.ksModuleDriveController, ModuleConstants.kvModuleDriveController, ModuleConstants.kaModuleDriveController);

    NetworkTable m_table;



    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel      The channel of the drive motor.
     * @param turningMotorChannel    The channel of the turning motor.
     * @param driveEncoderChannels   The channels of the drive encoder.
     * @param turningEncoderChannels The channels of the turning encoder.
     * @param driveEncoderReversed   Whether the drive encoder is reversed.
     * @param turningEncoderReversed Whether the turning encoder is reversed.
     */
    public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel,
            int turningEncoderChannels,
            double encoderOffset,
            String name) {
        m_driveMotor = new WPI_TalonFX(driveMotorChannel);
        m_turningMotor = new WPI_TalonFX(turningMotorChannel);

        m_turningEncoder = new WPI_CANCoder(turningEncoderChannels);

        m_table = NetworkTableInstance.getDefault().getTable("Swerve" + name);

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        m_turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        m_turningEncoder.configMagnetOffset(encoderOffset);
        m_turningEncoder.configSensorDirection(true);
        m_turningEncoder.setPositionToAbsolute();

        m_turningMotor.configRemoteFeedbackFilter(m_turningEncoder, 0);
        m_turningMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
        m_turningMotor.configSelectedFeedbackCoefficient(1);
        m_turningMotor.config_kP(0, ModuleConstants.kPModuleTurningController);

        m_driveMotor.configVoltageCompSaturation(10);
        m_driveMotor.configClosedloopRamp(0.5);
        m_driveMotor.configOpenloopRamp(0.5);

        m_desired_state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        m_name = name;

        SmartDashboard.putBoolean("Enable Driving", false);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                m_driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderDistancePerPulse * 10,
                getCancoderCurrentAngle());
    }

    public Rotation2d getCancoderCurrentAngle() {
        return new Rotation2d(Units.degreesToRadians(m_turningEncoder.getAbsolutePosition()));
    }

    public Rotation2d getTurningMotorAngle() {
        return Rotation2d.fromDegrees(Utils.falconToDegrees4096(m_turningMotor.getSelectedSensorPosition(), TURNING_GEAR_RATION));
    }

    public SwerveModuleState getDesiredState() {
        return m_desired_state;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        m_desired_state = CTREModuleState.optimize(desiredState, getState().angle);
        //SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningEncoderRadians()));

        double driveOutput = Utils.MPSToFalcon(m_desired_state.speedMetersPerSecond, ModuleConstants.kWheelDiameterMeters * Math.PI, DRIVE_GEAR_RATION);

        double turnOutput = Utils.degreesToFalcon2048(m_desired_state.angle.getDegrees(), TURNING_GEAR_RATION);

        // Debugging values
        m_table.getEntry("getSelectedSensorPosition").setNumber(m_turningMotor.getSelectedSensorPosition());
        m_table.getEntry("cancoderAngle").setNumber(getCancoderCurrentAngle().getDegrees());
        m_table.getEntry("motorAngle").setNumber(getTurningMotorAngle().getDegrees());
        m_table.getEntry("driveOutput").setNumber(driveOutput);
        m_table.getEntry("turnOutput").setNumber(turnOutput);


        if (SmartDashboard.getBoolean("Enable Driving", false)) {
            m_driveMotor.set(ControlMode.Velocity, driveOutput, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
            m_turningMotor.set(ControlMode.Position, turnOutput);
        }
    }

    public double getDriveMotorPercentage() {
        return m_driveMotor.getMotorOutputPercent();
    }

    public double getTurningMotorPercentage() {
        return m_turningMotor.getMotorOutputPercent();
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_driveMotor.setSelectedSensorPosition(0);
        m_turningEncoder.setPosition(0);
    }

    public String getName() {
        return m_name;
    }
}
