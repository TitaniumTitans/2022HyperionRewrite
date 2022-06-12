package org.titaniumtitans.lib.drivers;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;

public class EnhancedTalonFX extends TalonFX implements ITalonEnhanced {
    protected double m_lastSet = Double.NaN;
    protected TalonFXControlMode m_lastControlMode = null;
    protected Faults faults = new Faults();

    public EnhancedTalonFX(int id) {
        super(id);
        autoRetry(() -> clearStickyFaults(kTimeoutMs));
        defaultFrameIntervals();
    }

    public double getLastSet() {
        return m_lastSet;
    }

    /*
     * This must be overriden in the child class rather than ITalonEnhanced. The
     * reason for this is that interfaces cannot have non-static fields, and this
     * method needs m_lastSet and m_lastControlMode to work.
     */

    @Override
    public void set(TalonFXControlMode mode, double value) {
        if (value != m_lastSet || mode != m_lastControlMode) {
            m_lastSet = value;
            m_lastControlMode = mode;
            super.set(mode, value);
        }
    }

    public void handleFaults () {
        getFaults(faults);
        if (faults.hasAnyFault()) {
            DriverStation.reportError(String.format("Faults: %s", faults.toString()), false);
        }
    }
}