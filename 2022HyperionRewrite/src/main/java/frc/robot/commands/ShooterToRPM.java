package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterToRPM extends CommandBase{
    private final Shooter m_shooter;
    private final double rpm;

    public ShooterToRPM(Shooter shooter, Double rpm){
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
