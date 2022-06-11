package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

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
        if (m_shooter.atRPM(rpm)){
            m_indexer.driveKicker(0.75);
            m_indexer.driveMagazine(0.75);
        }
    }

    @Override
    public void end(boolean interrupted){
    }
    
    @Override
    public boolean isFinished(){
        return true;
    }
}
