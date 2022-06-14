package org.titaniumtitans.frc2022.commands;

import org.titaniumtitans.frc2022.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterToRPM extends CommandBase{
    private final Shooter m_shooter;
    private final double rpm;

    public ShooterToRPM(Shooter shooter, double rpm){
        m_shooter = shooter;
        this.rpm = rpm;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        m_shooter.shootAtVelocity(rpm);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return m_shooter.atRPM(rpm);
    }
    
}
