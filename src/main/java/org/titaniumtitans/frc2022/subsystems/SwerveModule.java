// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.titaniumtitans.frc2022.Constants.ModuleConstants;
import org.titaniumtitans.lib.Swerve.CTREModuleState;
import org.titaniumtitans.lib.Utils;

public class SwerveModule implements Sendable {


    // MK4i - L1
    private static final double TURNING_GEAR_RATION = (50.0 / 14.0) * (60.0 / 10.0);
    private static final double DRIVE_GEAR_RATION = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);


    private final WPI_TalonFX m_driveMotor;
    private final WPI_TalonFX m_turningMotor;

    private final CANCoder m_turningEncoder;

    private final String m_name;

    private SwerveModuleState m_desired_state;
    private double m_turnGoalTicks;
    private double m_driveGoalTicks;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ModuleConstants.ksModuleDriveController, ModuleConstants.kvModuleDriveController, ModuleConstants.kaModuleDriveController);

    double m_lastAngle;



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

        m_turningEncoder = new CANCoder(turningEncoderChannels);

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        m_turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        m_turningEncoder.configMagnetOffset(encoderOffset);
        m_turningEncoder.configSensorDirection(true);

        m_turningEncoder.configSensorDirection(false);
        m_turningEncoder.setPositionToAbsolute();

        m_turningMotor.setInverted(TalonFXInvertType.Clockwise);
        m_turningMotor.config_kP(0, ModuleConstants.kPModuleTurningController);
        m_turningMotor.configNeutralDeadband(0.1);
        setAbsoluteValue();

        m_driveMotor.configVoltageCompSaturation(10);
        m_driveMotor.configClosedloopRamp(0.5);
        m_driveMotor.configOpenloopRamp(0.5);
        m_driveMotor.configNeutralDeadband(0.1);

        m_desired_state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        m_name = name;
        m_lastAngle = getState().angle.getDegrees();

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
                getTurningMotorAngle());
    }

    public Rotation2d getCancoderCurrentAngle() {
        return new Rotation2d(Units.degreesToRadians(m_turningEncoder.getAbsolutePosition()));
    }

    public Rotation2d getTurningMotorAngle() {
        return Rotation2d.fromDegrees(Utils.falconToDegrees(m_turningMotor.getSelectedSensorPosition(), TURNING_GEAR_RATION));
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

        double driveOutput = Utils.MPSToFalcon(m_desired_state.speedMetersPerSecond, ModuleConstants.kWheelDiameterMeters * Math.PI, ModuleConstants.kDriveGearRatio);

        double turnOutput = Utils.degreesToFalcon2048(m_desired_state.angle.getDegrees(), ModuleConstants.kTurningGearRatio);

        if(driveOutput <= 0.05){
            //turnOutput = m_lastAngle;
            setAbsoluteValue();
        }

        // Debugging values
        m_table.getEntry("getSelectedSensorPosition").setNumber(m_turningMotor.getSelectedSensorPosition());
        m_table.getEntry("cancoderAngle").setNumber(getCancoderCurrentAngle().getDegrees());
        m_table.getEntry("motorAngle").setNumber(getTurningMotorAngle().getDegrees());
        m_table.getEntry("driveOutput").setNumber(driveOutput);
        m_table.getEntry("turnOutput").setNumber(turnOutput);


        m_driveGoalTicks = Utils.MPSToFalcon(m_desired_state.speedMetersPerSecond, ModuleConstants.kWheelDiameterMeters * Math.PI, DRIVE_GEAR_RATION);
        m_turnGoalTicks = Utils.degreesToFalcon(m_desired_state.angle.getDegrees(), TURNING_GEAR_RATION);

        if (SmartDashboard.getBoolean("Enable Driving", true)) {
            m_driveMotor.set(ControlMode.Velocity, m_driveGoalTicks, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
            m_turningMotor.set(ControlMode.Position, m_turnGoalTicks);

            m_lastAngle = turnOutput;
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

    public void setAbsoluteValue(){
        double absolutePosition = Utils.degreesToFalcon2048(getCancoderCurrentAngle().getDegrees(), ModuleConstants.kTurningGearRatio);
        m_turningMotor.setSelectedSensorPosition(absolutePosition);
    }

    public void setModuleAngle(double degrees){
        m_turningMotor.set(ControlMode.Position, Utils.degreesToFalcon2048(degrees, 21.42));

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("CancoderAngle", () -> getCancoderCurrentAngle().getDegrees(), null);
        builder.addDoubleProperty("TurnTicks", m_turningMotor::getSelectedSensorPosition, null);
        builder.addDoubleProperty("MotorAngle", () -> getTurningMotorAngle().getDegrees(), null);
        builder.addDoubleProperty("TurnGoalDegrees", () -> m_desired_state.angle.getDegrees(), null);
        builder.addDoubleProperty("TurnGoalTicks", () -> m_turnGoalTicks, null);
        builder.addDoubleProperty("DriveGoalMps", () -> m_desired_state.speedMetersPerSecond, null);
        builder.addDoubleProperty("DriveGoalTicks", () -> m_driveGoalTicks, null);

    }
}
