package org.titaniumtitans.frc2022.commands;

import org.titaniumtitans.frc2022.subsystems.Indexer;
import org.titaniumtitans.frc2022.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CargoShoot extends CommandBase{
    private final Shooter m_shooter;
    private final Indexer m_indexer;
    private final double rpm;

    public CargoShoot(Shooter shooter, Indexer indexer, double rpm){
        m_shooter = shooter;
        m_indexer = indexer;
        this.rpm = rpm;
        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize(){}

    @Override 
    public void execute(){
        m_shooter.shootAtVelocity(rpm);
        m_indexer.driveKicker(0.75);
        m_indexer.driveMagazine(0.75);
    }

    @Override
    public void end(boolean interrupted){
        m_indexer.driveMagazine(0.0);
        m_indexer.driveKicker(0.0);
        m_shooter.shootAtVelocity(0.0);
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }
}
