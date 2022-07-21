package org.titaniumtitans.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    //Shooter motors, both Falcon 500s
    private static TalonFX m_shooterR;
    private static TalonFX m_shooterL;
    private static ShooterLimelight m_limelight;

    private static SimpleMotorFeedforward m_shooterController;
    private static ShuffleboardTab m_shooterTab;
    private static double m_lastRPM;

    public Shooter(){
        //Config for shooter motors
        m_shooterR = new TalonFX(22);
        m_shooterR.setNeutralMode(NeutralMode.Coast);
        m_shooterR.configVoltageCompSaturation(10);
        m_shooterR.enableVoltageCompensation(true);
        m_shooterR.setInverted(true);

        m_shooterL = new TalonFX(21);
        m_shooterL.setNeutralMode(NeutralMode.Coast);
        m_shooterL.configVoltageCompSaturation(10);
        m_shooterL.enableVoltageCompensation(true);

        m_shooterController = new SimpleMotorFeedforward(0.1309, 0.114, 0);
        m_lastRPM = 0.0;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ShooterRPM", m_lastRPM);
    }

    public void shootAtVelocity(double velocity){
        m_lastRPM = velocity;
        velocity = velocity / 60.0;
        m_shooterR.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, m_shooterController.calculate(velocity) / 10.0);
        m_shooterL.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, m_shooterController.calculate(velocity) / 10.0);
    }

    public boolean atRPM(double velocity){
        return velocity <= (m_shooterR.getSelectedSensorVelocity() + m_shooterL.getSelectedSensorVelocity() / 2);
    }


    public double getRPMForDistance(){
        return m_limelight.calcRPM();
    }

}
