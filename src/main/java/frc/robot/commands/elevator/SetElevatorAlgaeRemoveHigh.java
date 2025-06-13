// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import frc.robot.testingdashboard.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorAlgaeRemoveHigh extends Command {
  /** Creates a new Level1. */

  Elevator m_Elevator;
  boolean m_set;
  boolean m_finished;
  int m_level;

  public SetElevatorAlgaeRemoveHigh() {
    super(Elevator.getInstance(), "Elevator", "AlgaeRemoveLow");
    m_Elevator = Elevator.getInstance();
    addRequirements(m_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_set = false;
    m_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_set) {
      m_Elevator.setElevatorTargetAngle(Constants.ElevatorConstants.kElevatorHighAlgaeRemove);
      m_Elevator.setShoulderTargetAngle(Constants.ElevatorConstants.kShoulderHighAlgaeRemove);
    }
    if (m_Elevator.inGoalPosition()) {
      m_finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
