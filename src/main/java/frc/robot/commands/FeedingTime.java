// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
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
  private WoS m_WoS;
  private Funnel m_Funnel;

  private boolean m_hungry;
  private boolean m_set;

  public FeedingTime() {
    super(Elevator.getInstance(), "Level", "FeedingTime");
    m_elevator = Elevator.getInstance();
    m_WoS = WoS.getInstance();
    m_Funnel = Funnel.getInstance();
    addRequirements(m_elevator, m_Funnel, m_WoS);

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
      m_Funnel.spinIn(Constants.FunnelConstants.kFunnelSpeed);
      m_WoS.spinIn(Constants.WoSConstants.kWoSSpeed);
      m_set = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Funnel.noSpin(0);
    m_WoS.noSpin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.inGoalPosition() && m_WoS.getCoralDetected();
  }
}
