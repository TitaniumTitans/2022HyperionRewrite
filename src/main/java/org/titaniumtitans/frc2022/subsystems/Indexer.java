// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.subsystems;

//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.titaniumtitans.frc2022.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

    public static final double intakeSpeed = 0.5;
    public static final double magazineSpeed = 0.75;
    public static final double kickerReverseSpeed = -0.5;

    //Drive motors for magazine and intake
    private final TalonSRX m_intake;
    private final TalonFX m_magazine;
    private final TalonSRX m_kicker;
    //private final Compressor m_compressor;

    //Pnuematics for intake
    //private final Compressor m_compressor;
    private final DoubleSolenoid m_solenoids;

    //IR sensors for ball detection
    private final DigitalInput m_lowIR;
    private final DigitalInput m_highIR;


    /** Creates a new ExampleSubsystem. */
    public Indexer() {
        //Create motor controller objects
        m_intake = new TalonSRX(IndexerConstants.intakeDriveID);
        m_magazine = new TalonFX(IndexerConstants.magazineDriveID);
        m_kicker = new TalonSRX(IndexerConstants.kickerDriveID);

        //Create Solenoid Objects
        m_solenoids = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, IndexerConstants.intakeOpenPort, IndexerConstants.intakeClosePort);
        m_solenoids.set(Value.kOff);

        m_lowIR = new DigitalInput(IndexerConstants.lowSensorDIOPort);
        m_highIR = new DigitalInput(IndexerConstants.highSensorDIOPort);

        //m_compressor = new Compressor(1, PneumaticsModuleType.REVPH);

    }

    public boolean getBottomSensor(){
        return !m_lowIR.get();
    }

    public boolean getHighSensor(){
        return !m_highIR.get();
    }

    public boolean magazineFull(){
        return !m_lowIR.get() && !m_highIR.get();
    }

    public void driveIntake(double power){
        m_intake.set(ControlMode.PercentOutput, power);
    }

    public void driveMagazine(double power){
        m_magazine.set(ControlMode.PercentOutput, power);
    }

    public void driveKicker(double power){
        m_kicker.set(ControlMode.PercentOutput, power);
    }

    //TODO update to a single method with one parameter
    public void extendIntake(){
        m_solenoids.set(Value.kForward);
    }

    public void retractIntake(){
        m_solenoids.set(Value.kReverse);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
