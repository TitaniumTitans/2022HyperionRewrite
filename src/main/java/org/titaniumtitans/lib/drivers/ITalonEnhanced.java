package org.titaniumtitans.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import org.titaniumtitans.lib.drivers.CTREUtil.ConfigCall;

public interface ITalonEnhanced extends IMotorControllerEnhanced {
    final static int kTimeoutMs = 100;
    final static int kFastFrameMs = 45;
    final static int kSlowFrameMs = 255;

    public default ITalonEnhanced autoRetry(ConfigCall talonConfigCall) {
        CTREUtil.autoRetry(() -> talonConfigCall.run());
        return this;
    }

    public default ITalonEnhanced setFeedbackIntervals(int intervalMs) {
        return this
                .autoRetry(() -> this.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_2_Feedback0, intervalMs, kTimeoutMs))
                .autoRetry(() -> this.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_Brushless_Current, intervalMs, kTimeoutMs));
    }

    public default ITalonEnhanced setControlIntervals(int intervalMs) {
        return this.autoRetry(() -> this.setControlFramePeriod(ControlFrame.Control_3_General, intervalMs));
    }

    public default ITalonEnhanced setAllStatusIntervals(int intervalMs) {
        for (StatusFrameEnhanced frame : StatusFrameEnhanced.values()) {
            this.autoRetry(() -> this.setStatusFramePeriod(frame, intervalMs, kTimeoutMs));
        }
        return this;
    }

    public default ITalonEnhanced defaultFrameIntervals() {
        return this.setAllStatusIntervals(kSlowFrameMs).setControlIntervals(kFastFrameMs);
    }
}