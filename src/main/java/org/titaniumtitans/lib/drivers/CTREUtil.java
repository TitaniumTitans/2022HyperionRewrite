package org.titaniumtitans.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import edu.wpi.first.wpilibj.DriverStation;

public class CTREUtil {
    interface ConfigCall {
        abstract ErrorCode run();
    }

    private static final int RETRY_COUNT = 3;
    private static final int RETRY_DELAY_MS = 50;

    static boolean hasError(ErrorCode err) {
        return err != ErrorCode.OK;
    }

    static void reportError(ErrorCode err, boolean warning) {
        String msg = String.format("Error: %s", err.toString());
        if (warning) {
            DriverStation.reportWarning(msg, false);
        } else {
            DriverStation.reportError(msg, false);
        }
    }

    static void autoRetry(ConfigCall configCall, String name) {
        int i;
        ErrorCode err;
        for (i = 1; hasError(err = configCall.run()) && i < RETRY_COUNT; i++) {
            DriverStation.reportWarning(String.format("%s: Try #%d failed: %s. Retrying...", name, i, err.toString()), false);
            try {
                Thread.sleep(RETRY_DELAY_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        if (i >= RETRY_COUNT) {
            DriverStation.reportError(String.format("%s: Try #%d failed: %s. Maximum retry count exceeded.", name, i, err.toString()), false);
        } else if (i > 1) {
            DriverStation.reportWarning(String.format("%s: Try #%d succeeded!", name, i), false);
        }
    }
}