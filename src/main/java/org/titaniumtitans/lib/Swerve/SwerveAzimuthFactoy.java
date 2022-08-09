package org.titaniumtitans.lib.Swerve;

import javax.naming.ldap.ControlFactory;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class SwerveAzimuthFactoy {
    static int kTimeoutMs = 100;


    static NeutralMode kNeutralMode = NeutralMode.Brake;
    static double kDeadBanding = 0.04;

    static boolean kInverted = false;
    static boolean kSensorPhase = false;

    static int kControlFrameMs = 20;
    static int kMotionControlFrameMs = 100;
    static int kGeneralStatusFrameMs = 20;

    static double kVoltageComp = 10.0;


    public static TalonFX createAzimuthTalon(int talonPort){
        TalonFX talon = new TalonFX(talonPort);

        talon.changeMotionControlFramePeriod(kMotionControlFrameMs);

        talon.setInverted(kInverted);
        talon.setSensorPhase(kSensorPhase);

        talon.configVoltageCompSaturation(kVoltageComp, kTimeoutMs);
        talon.enableVoltageCompensation(true);

        talon.configNeutralDeadband(kDeadBanding, kTimeoutMs);
        talon.setNeutralMode(kNeutralMode);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 
            kGeneralStatusFrameMs,
            kTimeoutMs);

        talon.setControlFramePeriod(ControlFrame.Control_3_General, 
            kControlFrameMs);

        return talon;
    }
    
}
