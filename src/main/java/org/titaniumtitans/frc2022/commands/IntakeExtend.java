// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.commands;

import org.titaniumtitans.frc2022.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeExtend extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Indexer m_indexer;

    /**
     * Creates a new IntakeExtend command.
     *
     * @param indexer A {@link Indexer} object represent the indexer subsystem.
     */
    public IntakeExtend(Indexer indexer) {
        m_indexer = indexer;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_indexer.magazineFull()) {

            // m_indexer.driveMagazine(Indexer.magazineSpeed);
        } else {
            m_indexer.extendIntake();
            m_indexer.driveIntake(Indexer.intakeSpeed);
            m_indexer.driveMagazine(Indexer.magazineSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_indexer.retractIntake();
        m_indexer.driveIntake(0.0);
        m_indexer.driveMagazine(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
