// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Indexer extends SubsystemBase {

  private static final double intakeSpeed = 1;
  private static final double magazineSpeed = 0.75;
  private static final double kickerReverseSpeed = -0.5;

  //Drive motors for magazine and intake
  private final TalonSRX m_intake;
  private final TalonFX m_magazine;
  private final TalonFX m_kicker;

  //Pnuematics for intake
  private final DoubleSolenoid m_solenoids;

  //IR sensors for ball detection
  private final DigitalInput m_lowIR;
  private final DigitalInput m_highIR;


  /** Creates a new ExampleSubsystem. */
  public Indexer() {
    //Create motor controller objects
    m_intake = new TalonSRX(16);
    m_magazine = new TalonFX(17);
    m_kicker = new TalonFX(18);

    //Create Solenoid Objects
    m_solenoids = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
    m_solenoids.set(Value.kOff);

    m_lowIR = new DigitalInput(1);
    m_highIR = new DigitalInput(2);

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

  public void extendIntake(){
    m_solenoids.set(Value.kForward);
  }

  public void rectractIntake(){
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
