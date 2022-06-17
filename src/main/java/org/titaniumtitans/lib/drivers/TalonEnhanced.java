package org.titaniumtitans.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import org.titaniumtitans.lib.drivers.CTREUtil.*;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * This is my second attempt to create a wrapper class for CTRE motor
 * controllers and add error checking, "lazy" setters (that only run when a
 * value changes), and some other convenience methods.
 * 
 * At the moment, it's not <em>technically</em> a drop-in replacement for
 * TalonFX or TalonSRX (some of those class's methods that aren't in
 * BaseMotorController are missing), but it's pretty close.
 */
public class TalonEnhanced {
    private final BaseMotorController m_motorController;

    // TODO Consider moving these constants somewhere else
    final static private int kTimeoutMs = 100;
    final static private int kFastFrameMs = 45;
    final static private int kSlowFrameMs = 255;

    // TODO Set common parameters in constructor, add more
    // TODO overloaded constructor methods
    public TalonEnhanced(BaseMotorController talon) {
        m_motorController = talon;
        this.setAllStatusIntervals(255)
                .setControlIntervals(45)
                .setFeedbackIntervals(20);
    }

    public BaseMotorController getController () {
        return m_motorController;
    }

    // TODO Change to handleError and change behavior depending on error type
    public TalonEnhanced autoRetry(ConfigCall talonConfigCall) {
        CTREUtil.autoRetry(talonConfigCall, String.format("%s(%d)", m_motorController.getClass().getSimpleName(), this.getDeviceID()));
        return this;
    }

    private double m_lastDemand0 = Double.NaN;
    private ControlMode m_lastControlMode = null;

    /**
     * Same idea as 254's "LazyTalonFX" class. The
     * {@link BaseMotorController}'s set method is only called when demand0 is
     * changed.
     */
    public TalonEnhanced set(ControlMode controlMode, double demand0) {
        if (controlMode != m_lastControlMode || demand0 != m_lastDemand0) {
            m_motorController.set(controlMode, demand0);
            m_lastControlMode = controlMode;
            m_lastDemand0 = demand0;
        }
        return this;
    }

    private DemandType m_lastDemandType;
    private double m_lastDemand1;

    /**
     * Same idea as 254's "LazyTalonFX" class. The
     * {@link BaseMotorController}'s set method is only called when demand0 is
     * changed.
     */
    public TalonEnhanced set(ControlMode controlMode, double demand0, DemandType demandType, double demand1) {
        if (controlMode != m_lastControlMode ||
                demand0 != m_lastDemand0 ||
                demandType != m_lastDemandType ||
                demand1 != m_lastDemand1) {
            m_motorController.set(controlMode, demand0, demandType, demand1);
            m_lastControlMode = controlMode;
            m_lastDemand0 = demand0;
            m_lastDemandType = demandType;
            m_lastDemand1 = demand1;
        }
        return this;
    }

    public TalonEnhanced neutralOutput() {
        m_motorController.neutralOutput();
        return this;
    }

    public TalonEnhanced setNeutralMode(NeutralMode neutralMode) {
        m_motorController.setNeutralMode(neutralMode);
        return this;
    }

    public TalonEnhanced setSensorPhase(boolean PhaseSensor) {
        m_motorController.setSensorPhase(PhaseSensor);
        return this;
    }

    public TalonEnhanced setInverted(boolean invert) {
        m_motorController.setInverted(invert);

        return this;
    }

    public TalonEnhanced setInverted(InvertType invertType) {
        m_motorController.setInverted(invertType);

        return this;
    }

    public boolean getInverted() {
        return m_motorController.getInverted();
    }

    public TalonEnhanced configOpenloopRamp(double secondsFromNeutralToFull) {
        return this.autoRetry(() -> m_motorController.configOpenloopRamp(secondsFromNeutralToFull, kTimeoutMs));
    }

    public TalonEnhanced configClosedloopRamp(double secondsFromNeutralToFull) {
        return this.autoRetry(() -> m_motorController.configClosedloopRamp(secondsFromNeutralToFull, kTimeoutMs));
    }

    public TalonEnhanced configPeakOutputForward(double percentOut) {
        return this.autoRetry(() -> m_motorController.configPeakOutputForward(percentOut, kTimeoutMs));
    }

    public TalonEnhanced configPeakOutputReverse(double percentOut) {
        return this.autoRetry(() -> m_motorController.configPeakOutputReverse(percentOut, kTimeoutMs));
    }

    public TalonEnhanced configNominalOutputForward(double percentOut) {
        return this.autoRetry(() -> m_motorController.configNominalOutputForward(percentOut, kTimeoutMs));
    }

    public TalonEnhanced configNominalOutputReverse(double percentOut) {
        return this.autoRetry(() -> m_motorController.configNominalOutputReverse(percentOut, kTimeoutMs));
    }

    public TalonEnhanced configNeutralDeadband(double percentDeadband) {
        return this.autoRetry(() -> m_motorController.configNeutralDeadband(percentDeadband, kTimeoutMs));
    }

    public TalonEnhanced configVoltageCompSaturation(double voltage) {
        return this.autoRetry(() -> m_motorController.configVoltageCompSaturation(voltage, kTimeoutMs));
    }

    public TalonEnhanced configVoltageMeasurementFilter(int filterWindowSamples) {
        return this.autoRetry(() -> m_motorController.configVoltageMeasurementFilter(filterWindowSamples, kTimeoutMs));
    }

    public TalonEnhanced enableVoltageCompensation(boolean enable) {
        m_motorController.enableVoltageCompensation(enable);
        return this;
    }

    public double getBusVoltage() {
        return m_motorController.getBusVoltage();
    }

    public double getMotorOutputPercent() {
        return m_motorController.getMotorOutputPercent();
    }

    public double getMotorOutputVoltage() {
        return m_motorController.getMotorOutputVoltage();
    }

    public double getTemperature() {
        return m_motorController.getTemperature();
    }

    public TalonEnhanced configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx) {
        return this.autoRetry(() -> m_motorController.configSelectedFeedbackSensor(feedbackDevice, pidIdx, kTimeoutMs));
    }

    public TalonEnhanced configSelectedFeedbackCoefficient(double coefficient, int pidIdx) {
        return this
                .autoRetry(() -> m_motorController.configSelectedFeedbackCoefficient(coefficient, pidIdx, kTimeoutMs));
    }

    public TalonEnhanced configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource,
            int remoteOrdinal) {
        return this.autoRetry(
                () -> m_motorController.configRemoteFeedbackFilter(deviceID, remoteSensorSource, remoteOrdinal,
                        kTimeoutMs));
    }

    public TalonEnhanced configRemoteFeedbackFilter(CANCoder canCoderRef, int remoteOrdinal) {
        return this
                .autoRetry(() -> m_motorController.configRemoteFeedbackFilter(canCoderRef, remoteOrdinal, kTimeoutMs));
    }

    public TalonEnhanced configRemoteFeedbackFilter(BaseTalon talonRef, int remoteOrdinal) {
        return this.autoRetry(() -> m_motorController.configRemoteFeedbackFilter(talonRef, remoteOrdinal, kTimeoutMs));
    }

    public TalonEnhanced configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice) {
        return this.autoRetry(() -> m_motorController.configSensorTerm(sensorTerm, feedbackDevice, kTimeoutMs));
    }

    public double getSelectedSensorPosition(int pidIdx) {
        return m_motorController.getSelectedSensorPosition(pidIdx);
    }

    public double getSelectedSensorVelocity(int pidIdx) {
        return m_motorController.getSelectedSensorVelocity(pidIdx);
    }

    public TalonEnhanced setSelectedSensorPosition(double sensorPos, int pidIdx) {
        return this.autoRetry(() -> m_motorController.setSelectedSensorPosition(sensorPos, pidIdx, kTimeoutMs));
    }

    public TalonEnhanced setControlFramePeriod(ControlFrame frame, int periodMs) {
        return this.autoRetry(() -> m_motorController.setControlFramePeriod(frame, periodMs));
    }

    public TalonEnhanced setStatusFramePeriod(StatusFrame frame, int periodMs) {
        return this.autoRetry(() -> m_motorController.setStatusFramePeriod(frame, periodMs, kTimeoutMs));
    }

    public int getStatusFramePeriod(StatusFrame frame) {
        return m_motorController.getStatusFramePeriod(frame, kTimeoutMs);
    }

    public TalonEnhanced overrideLimitSwitchesEnable(boolean enable) {
        m_motorController.overrideLimitSwitchesEnable(enable);
        return this;
    }

    public TalonEnhanced configForwardSoftLimitThreshold(double forwardSensorLimit) {
        return this.autoRetry(() -> m_motorController.configForwardSoftLimitThreshold(forwardSensorLimit, kTimeoutMs));
    }

    public TalonEnhanced configReverseSoftLimitThreshold(double reverseSensorLimit) {
        return this.autoRetry(() -> m_motorController.configReverseSoftLimitThreshold(reverseSensorLimit, kTimeoutMs));
    }

    public TalonEnhanced configForwardSoftLimitEnable(boolean enable) {
        return this.autoRetry(() -> m_motorController.configForwardSoftLimitEnable(enable, kTimeoutMs));
    }

    public TalonEnhanced configReverseSoftLimitEnable(boolean enable) {
        return this.autoRetry(() -> m_motorController.configReverseSoftLimitEnable(enable, kTimeoutMs));
    }

    public TalonEnhanced overrideSoftLimitsEnable(boolean enable) {
        m_motorController.overrideLimitSwitchesEnable(enable);
        return this;
    }

    // TODO Create some PIDF convenience functions

    private double m_last_kP = Double.NaN;

    public TalonEnhanced config_kP(int slotIdx, double value) {
        if (m_last_kP != value) {
            m_last_kP = value;
            return this.autoRetry(() -> m_motorController.config_kP(slotIdx, value, kTimeoutMs));
        } else
            return this;
    }

    private double m_last_kI = Double.NaN;

    public TalonEnhanced config_kI(int slotIdx, double value) {
        if (m_last_kI != value) {
            m_last_kI = value;
            return this.autoRetry(() -> m_motorController.config_kI(slotIdx, value, kTimeoutMs));
        } else
            return this;
    }

    private double m_last_kD = Double.NaN;

    public TalonEnhanced config_kD(int slotIdx, double value) {
        if (m_last_kD != value) {
            m_last_kD = value;
            return this.autoRetry(() -> m_motorController.config_kD(slotIdx, value, kTimeoutMs));
        } else
            return this;
    }

    private double m_last_kF = Double.NaN;

    public TalonEnhanced config_kF(int slotIdx, double value) {
        if (m_last_kF != value) {
            m_last_kF = value;
            return this.autoRetry(() -> m_motorController.config_kF(slotIdx, value, kTimeoutMs));
        } else
            return this;
    }

    private double m_lastIzone = Double.NaN;

    public TalonEnhanced config_IntegralZone(int slotIdx, double izone) {
        if (m_lastIzone != izone) {
            m_lastIzone = izone;
            return this.autoRetry(() -> m_motorController.config_IntegralZone(slotIdx, izone, kTimeoutMs));
        } else
            return this;
    }

    private double m_lastAllowableCloseLoopError = Double.NaN;

    public TalonEnhanced configAllowableClosedloopError(int slotIdx, double allowableCloseLoopError) {
        if (m_lastAllowableCloseLoopError != allowableCloseLoopError) {
            m_lastAllowableCloseLoopError = allowableCloseLoopError;
            return this.autoRetry(
                    () -> m_motorController.configAllowableClosedloopError(slotIdx, allowableCloseLoopError,
                            kTimeoutMs));
        } else
            return this;
    }

    private double m_lastMaxIaccum = Double.NaN;

    public TalonEnhanced configMaxIntegralAccumulator(int slotIdx, double maxIaccum) {
        if (m_lastMaxIaccum != maxIaccum) {
            m_lastMaxIaccum = maxIaccum;
            return this.autoRetry(() -> m_motorController.configMaxIntegralAccumulator(slotIdx, maxIaccum, kTimeoutMs));
        } else
            return this;
    }

    public TalonEnhanced configClosedLoopPeakOutput(int slotIdx, double percentOut) {
        return this.autoRetry(() -> m_motorController.configClosedLoopPeakOutput(slotIdx, percentOut, kTimeoutMs));
    }

    public TalonEnhanced configClosedLoopPeriod(int slotIdx, int loopTimeMs) {
        return this.autoRetry(() -> m_motorController.configClosedLoopPeriod(slotIdx, loopTimeMs, kTimeoutMs));
    }

    public TalonEnhanced configAuxPIDPolarity(boolean invert) {
        return this.autoRetry(() -> m_motorController.configAuxPIDPolarity(invert, kTimeoutMs));
    }

    public TalonEnhanced setIntegralAccumulator(double iaccum, int pidIdx) {
        return this.autoRetry(() -> m_motorController.setIntegralAccumulator(iaccum, pidIdx, kTimeoutMs));
    }

    public double getClosedLoopError(int pidIdx) {
        return m_motorController.getClosedLoopError(pidIdx);
    }

    public double getIntegralAccumulator(int pidIdx) {
        return m_motorController.getIntegralAccumulator(pidIdx);
    }

    public double getErrorDerivative(int pidIdx) {
        return m_motorController.getErrorDerivative(pidIdx);
    }

    public TalonEnhanced selectProfileSlot(int slotIdx, int pidIdx) {
        m_motorController.selectProfileSlot(slotIdx, pidIdx);
        return this;
    }

    public double getClosedLoopTarget(int pidIdx) {
        return m_motorController.getClosedLoopTarget(pidIdx);
    }

    public double getActiveTrajectoryPosition() {
        return m_motorController.getActiveTrajectoryPosition();
    }

    public double getActiveTrajectoryVelocity() {
        return m_motorController.getActiveTrajectoryVelocity();
    }

    // TODO Write motion magic convenience methods.

    private double m_lastCruiseVelocity = Double.NaN;

    public TalonEnhanced configMotionCruiseVelocity(double cruiseVelocity) {
        if (m_lastCruiseVelocity != cruiseVelocity) {
            m_lastCruiseVelocity = cruiseVelocity;
            return this.autoRetry(() -> m_motorController.configMotionCruiseVelocity(cruiseVelocity, kTimeoutMs));
        } else
            return this;
    }

    private double m_lastAccel = Double.NaN;

    public TalonEnhanced configMotionAcceleration(double accel) {
        if (m_lastAccel != accel) {
            m_lastAccel = accel;
            return this.autoRetry(() -> m_motorController.configMotionAcceleration(accel, kTimeoutMs));
        } else
            return this;
    }

    public TalonEnhanced configMotionSCurveStrength(int curveStrength) {
        return this.autoRetry(() -> m_motorController.configMotionSCurveStrength(curveStrength, kTimeoutMs));
    }

    public TalonEnhanced configMotionProfileTrajectoryPeriod(int baseTrajDurationMs) {
        return this
                .autoRetry(() -> m_motorController.configMotionProfileTrajectoryPeriod(baseTrajDurationMs, kTimeoutMs));
    }

    public TalonEnhanced clearMotionProfileTrajectories() {
        return this.autoRetry(m_motorController::clearMotionProfileTrajectories);
    }

    public int getMotionProfileTopLevelBufferCount() {
        return m_motorController.getMotionProfileTopLevelBufferCount();
    }

    public TalonEnhanced pushMotionProfileTrajectory(TrajectoryPoint trajPt) {
        return this.autoRetry(() -> m_motorController.pushMotionProfileTrajectory(trajPt));
    }

    public boolean isMotionProfileTopLevelBufferFull() {
        return m_motorController.isMotionProfileTopLevelBufferFull();
    }

    public TalonEnhanced processMotionProfileBuffer() {
        m_motorController.processMotionProfileBuffer();

        return this;
    }

    public TalonEnhanced getMotionProfileStatus(MotionProfileStatus statusToFill) {
        return this.autoRetry(() -> m_motorController.getMotionProfileStatus(statusToFill));
    }

    public TalonEnhanced clearMotionProfileHasUnderrun() {
        return this.autoRetry(() -> m_motorController.clearMotionProfileHasUnderrun(kTimeoutMs));
    }

    public TalonEnhanced changeMotionControlFramePeriod(int periodMs) {
        return this.autoRetry(() -> m_motorController.changeMotionControlFramePeriod(periodMs));
    }

    public ErrorCode getLastError() {
        return m_motorController.getLastError();
    }

    // TODO Implement automatic fault monitoring/handling. Perhaps divide faults
    // TODO into "categories" such as limits/soft limits, hardware/api issues, and
    // TODO non-critical "warning" faults.
    public TalonEnhanced getFaults(Faults toFill) {
        return this.autoRetry(() -> m_motorController.getFaults(toFill));
    }

    public TalonEnhanced getStickyFaults(StickyFaults toFill) {
        return this.autoRetry(() -> m_motorController.getStickyFaults(toFill));
    }

    public TalonEnhanced clearStickyFaults() {
        return this.autoRetry(() -> m_motorController.clearStickyFaults(kTimeoutMs));
    }

    public int getFirmwareVersion() {
        return m_motorController.getFirmwareVersion();
    }

    // TODO Automatically re-instate config when this happens. Perhaps save a live
    // TODO "copy" of the config in a variable?
    public boolean hasResetOccurred() {
        return m_motorController.hasResetOccurred();
    }

    public TalonEnhanced configSetCustomParam(int newValue, int paramIndex) {
        return this.autoRetry(() -> m_motorController.configSetCustomParam(newValue, paramIndex, kTimeoutMs));
    }

    public int configGetCustomParam(int paramIndex) {
        return m_motorController.configGetCustomParam(paramIndex, kTimeoutMs);
    }

    public TalonEnhanced configSetParameter(ParamEnum param, double value, int subValue, int ordinal) {
        return this.autoRetry(() -> m_motorController.configSetParameter(param, value, subValue, ordinal, kTimeoutMs));
    }

    public TalonEnhanced configSetParameter(int param, double value, int subValue, int ordinal) {
        return this.autoRetry(() -> m_motorController.configSetParameter(param, value, subValue, ordinal, kTimeoutMs));
    }

    public double configGetParameter(ParamEnum paramEnum, int ordinal) {
        return m_motorController.configGetParameter(paramEnum, ordinal, kTimeoutMs);
    }

    public double configGetParameter(int paramEnum, int ordinal) {
        return m_motorController.configGetParameter(paramEnum, ordinal, kTimeoutMs);
    }

    public int getBaseID() {
        return m_motorController.getBaseID();
    }

    public int getDeviceID() {
        return m_motorController.getDeviceID();
    }

    public ControlMode getControlMode() {
        return m_motorController.getControlMode();
    }

    public TalonEnhanced follow(IMotorController masterToFollow) {
        m_motorController.follow(masterToFollow);

        return this;
    }

    public TalonEnhanced follow(TalonEnhanced masterToFollow) {
        return this.follow(masterToFollow.m_motorController);
    }

    public TalonEnhanced configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx) {
        return this.autoRetry(() -> m_motorController.configSelectedFeedbackSensor(feedbackDevice, pidIdx, kTimeoutMs));
    }

    public TalonEnhanced setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs) {
        // TODO Create custom frame enum which better represents common use cases
        return this.autoRetry(() -> m_motorController.setStatusFramePeriod(frame.value, periodMs, kTimeoutMs));
    }

    public int getStatusFramePeriod(StatusFrameEnhanced frame) {
        // TODO Create custom frame enum which better represents common use cases
        return m_motorController.getStatusFramePeriod(frame, kTimeoutMs);
    }

    // TODO Refactor these methods using custom frame enum
    public TalonEnhanced setFeedbackIntervals(int intervalMs) {
        return this
                .autoRetry(() -> m_motorController.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_2_Feedback0.value, intervalMs, kTimeoutMs))
                .autoRetry(() -> m_motorController.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_Brushless_Current.value, intervalMs, kTimeoutMs));
    }

    public TalonEnhanced setControlIntervals(int intervalMs) {
        return this
                .autoRetry(() -> m_motorController.setControlFramePeriod(ControlFrame.Control_3_General, intervalMs));
    }

    public TalonEnhanced setAllStatusIntervals(int intervalMs) {
        for (StatusFrameEnhanced frame : StatusFrameEnhanced.values()) {
            this.autoRetry(() -> m_motorController.setStatusFramePeriod(frame.value, intervalMs, kTimeoutMs));
        }
        return this;
    }

    public TalonEnhanced defaultFrameIntervals() {
        return this.setAllStatusIntervals(kSlowFrameMs).setControlIntervals(kFastFrameMs);
    }

    public double getSupplyCurrent() {
        if (m_motorController instanceof BaseTalon) {
            BaseTalon m_talon = (BaseTalon) m_motorController;
            return m_talon.getSupplyCurrent();
        } else {
            DriverStation.reportError(String.format(
                    "CTREMotorController(%s): Current reporting is only available for Talons. Returning 0.0.",
                    m_motorController.getDeviceID()), false);
        }
        return 0.0;
    }

    public double getStatorCurrent() {
        if (m_motorController instanceof BaseTalon) {
            BaseTalon m_talon = (BaseTalon) m_motorController;
            return m_talon.getStatorCurrent();
        } else {
            DriverStation.reportError(String.format(
                    "CTREMotorController(%s): Current reporting is only available for Talons. Returning 0.0.",
                    m_motorController.getDeviceID()), false);
        }
        return 0.0;
    }

    public TalonEnhanced configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitCfg) {
        if (m_motorController instanceof BaseTalon) {
            BaseTalon m_talon = (BaseTalon) m_motorController;
            return this.autoRetry(() -> m_talon.configSupplyCurrentLimit(currLimitCfg, kTimeoutMs));
        } else {
            DriverStation.reportError(String.format(
                    "CTREMotorController(%s): Supply current limiting is only available for Talons.",
                    m_motorController.getDeviceID()), false);
        }
        return this;
    }

    public TalonEnhanced configStatorCurrentLimit(StatorCurrentLimitConfiguration currLimitCfg) {
        if (m_motorController instanceof TalonFX) {
            TalonFX m_talon = (TalonFX) m_motorController;
            return this.autoRetry(() -> m_talon.configStatorCurrentLimit(currLimitCfg, kTimeoutMs));
        } else {
            DriverStation.reportError(String.format(
                    "CTREMotorController(%s): Stator current limiting is only available for TalonFXs.",
                    m_motorController.getDeviceID()), false);
        }
        return this;
    }

    public TalonEnhanced configVelocityMeasurementPeriod(SensorVelocityMeasPeriod period) {
        return this.autoRetry(() -> m_motorController.configVelocityMeasurementPeriod(period, kTimeoutMs));
    }

    public TalonEnhanced configVelocityMeasurementWindow(int windowSize) {
        return this.autoRetry(() -> m_motorController.configVelocityMeasurementWindow(windowSize, kTimeoutMs));
    }

    public TalonEnhanced configForwardLimitSwitchSource(RemoteLimitSwitchSource type,
            LimitSwitchNormal normalOpenOrClose, int deviceID) {
        return this
                .autoRetry(() -> m_motorController.configForwardLimitSwitchSource(type, normalOpenOrClose, deviceID,
                        kTimeoutMs));
    }

    public TalonEnhanced configReverseLimitSwitchSource(RemoteLimitSwitchSource type,
            LimitSwitchNormal normalOpenOrClose, int deviceID) {
        return this
                .autoRetry(() -> m_motorController.configReverseLimitSwitchSource(type, normalOpenOrClose, deviceID,
                        kTimeoutMs));
    }

    public TalonEnhanced configForwardLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose) {
        return this
                .autoRetry(() -> m_motorController.configForwardLimitSwitchSource(type, normalOpenOrClose, kTimeoutMs));
    }

    // TODO Fix this.
    public TalonEnhanced configReverseLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose) {
        return this;
        // return this
        // .autoRetry(() -> m_motorController.configReverseLimitSwitchSource(type,
        // normalOpenOrClose, kTimeoutMs));

    }
}