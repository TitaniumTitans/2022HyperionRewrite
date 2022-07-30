package org.titaniumtitans.frc2022.commands;

import org.titaniumtitans.frc2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberPIDControl extends CommandBase {
    Climber m_climber;
    boolean isUp;

    public ClimberPIDControl(Climber m_climber, boolean up){
        this.m_climber = m_climber;
        isUp = up;
        addRequirements(this.m_climber);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        if(isUp == true){
            m_climber.climberPidControl(m_climber.m_leftMaxHeight.getValue(), m_climber.m_rightMaxHeight.getValue());
        }
        if(isUp == false){
            m_climber.climberPidControl(500, 500);
        }
    }
    
    @Override
    public void end(boolean interupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
