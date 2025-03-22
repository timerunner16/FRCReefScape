// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.WoS.Consume;
import frc.robot.commands.funnel.Implode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.WoS;
import frc.robot.testingdashboard.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeedingTime extends Command {
  /** Creates a new FeedingTime. */
  private Elevator m_elevator;
  private Implode m_implode;
  private Consume m_consume;

  private boolean m_hungry;
  private boolean m_set;

  public FeedingTime() {
    super(Elevator.getInstance(), "Level", "FeedingTime");
    m_elevator = Elevator.getInstance();
    addRequirements(m_elevator);

    m_implode = new Implode();
    m_consume = new Consume();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setElevatorTargetLevel(0);
    m_elevator.setShoulderTargetLevel(0);

    m_hungry = false;
    m_set = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hungry = m_elevator.inGoalPosition();
    if (m_hungry && !m_set) {
      m_implode.schedule();
      m_consume.schedule();
      m_set = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_implode.cancel();
    m_consume.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
