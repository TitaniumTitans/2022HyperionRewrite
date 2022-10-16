package org.titaniumtitans.frc2022.commands;

import org.titaniumtitans.frc2022.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterToRPM extends CommandBase{
    private final Shooter m_shooter;
    private double m_rpm;

    public ShooterToRPM(Shooter shooter, double rpm){
        m_shooter = shooter;
        m_rpm = m_shooter.getRPMForDistance();
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        m_shooter.shootAtVelocity(m_rpm);
        m_rpm = m_shooter.getRPMForDistance();
    }

    @Override
    public void end(boolean interrupted){
        m_shooter.shootAtVelocity(0.0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
