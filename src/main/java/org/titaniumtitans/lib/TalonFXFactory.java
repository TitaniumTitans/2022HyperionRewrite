package org.titaniumtitans.lib;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.DriverStation;
import org.titaniumtitans.lib.drivers.CTREUtil;

public class TalonFXFactory {

    public static class TalonFXConfiguration {
        public int defaultTimeout = 100;

        public NeutralMode neutralMode = NeutralMode.Brake;
        public double deadBand = 0.04;
        public InvertType inverted = InvertType.None;
        public boolean sensorPhase = false;

        public int controlFramePeriodMs = 40;
        public int statusFramePeriodMs = 100;
        public int motionControlFramePeriodMs = 40;

        public double voltageCompensationSaturationVoltage = 10;
        public boolean voltageCompensationEnabled = true;

        public double PID_P_Constant = 0;
        public double PID_I_Constant = 0;
        public double PID_D_Constant = 0;
        public double PID_I_ZONE = 0;

        public double magicCruiseVelocity = 4000;
        public double magicAcceleration = 4000;

        public SensorInitializationStrategy initializationStrategy = SensorInitializationStrategy.BootToZero;
    }

    public static TalonFX createTalonFX(int canId) {
        return TalonFXFactory.createTalonFX(canId, new TalonFXConfiguration());
    }

    public static TalonFX createTalonFX(int canId, TalonFXConfiguration configuration) {
        TalonFX talon = new TalonFX(canId);

        reportIfError(talon.configFactoryDefault(configuration.defaultTimeout), canId, "Talon failed to factory default");
        reportIfError(talon.changeMotionControlFramePeriod(configuration.motionControlFramePeriodMs), canId, "Failed to set Motion Control Frame Period");

        // Set the 'phase' of the sensor first, so it matches the motor. The 'phase' is better known as the sensor direction.
        talon.setSensorPhase(configuration.sensorPhase);
        // Inverting the motor will also invert the sensor. Use setSensorPhase **first** to match the motor to the sensor.
        talon.setInverted(configuration.inverted);

        // Configure the voltage compensation AKA the max effective voltage applied to the motor
        if(configuration.voltageCompensationEnabled) {
            reportIfError(talon.configVoltageCompSaturation(configuration.voltageCompensationSaturationVoltage, configuration.defaultTimeout), canId, "Failed to set the voltage-compensation saturation-voltage.");
            talon.enableVoltageCompensation(configuration.voltageCompensationEnabled);
        }

        // Configure the neutral deadband
        reportIfError(talon.configNeutralDeadband(configuration.deadBand, configuration.defaultTimeout), canId, "Failed to set neutral deadband");
        talon.setNeutralMode(configuration.neutralMode);

        // Set the frame period for all the status frames
        if(configuration.statusFramePeriodMs != 0) {
            // TODO: Might not be a great idea to configure ALL the values
            // Status frame 9 might not be real/used
            for (StatusFrameEnhanced frame : StatusFrameEnhanced.values()) {
                CTREUtil.ConfigCall configCall = () -> talon.setStatusFramePeriod(frame, configuration.statusFramePeriodMs, configuration.defaultTimeout);
                reportIfError(CTREUtil.autoRetry(configCall), canId, "Failed to configure status frame \"" + frame + "\"");
            }
        }

        /*
         * Set the control frame period. By default, this is 10ms, but with many motors on the CANBUS, this can cause problems because of the CAN utilization
         * CTRE stated 50ms is fine, especially for followers, but 40-45ms will give each status 2 tries before the motor reaches the hard 100ms timeout limit
         * https://www.chiefdelphi.com/t/neos-vs-falcon-500s-for-mk4-sds-swerve/410828/43
         */
        reportIfError(talon.setControlFramePeriod(ControlFrame.Control_3_General, configuration.controlFramePeriodMs), canId, "Failed to set status frame period of type 1");

        // Configure PID constants
        reportIfError(talon.config_kP(0, configuration.PID_P_Constant), canId, "Failed to set PID P constant");
        reportIfError(talon.config_kI(0, configuration.PID_I_Constant), canId, "Failed to set PID I constant");
        reportIfError(talon.config_kD(0, configuration.PID_D_Constant), canId, "Failed to set PID D constant");
        reportIfError(talon.config_IntegralZone(0, configuration.PID_I_ZONE, configuration.defaultTimeout), canId, "Failed to set PID integral zone");

        // Set maximum rates for velocity and acceleration
        reportIfError(talon.configMotionCruiseVelocity(configuration.magicCruiseVelocity, configuration.defaultTimeout), canId, "Failed to set Magic Motion Peak Target Velocity");
        reportIfError(talon.configMotionAcceleration(configuration.magicAcceleration, configuration.defaultTimeout), canId, "Failed to set Magic Motion Peak Target Acceleration");

        reportIfError(talon.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, configuration.defaultTimeout), canId, "Failed to set integrated sensor initialization strategy");

        return talon;
    }

    private static void reportIfError(ErrorCode code, int canId, String error) {
        if(code != ErrorCode.OK) {
            DriverStation.reportError("Config Error (CAN ID " + canId + "): " + error, false);
        }
    }

}
