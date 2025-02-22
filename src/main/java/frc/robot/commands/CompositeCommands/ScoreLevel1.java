// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CompositeCommands;

import java.util.logging.Level;

import frc.robot.subsystems.WoS;
import frc.robot.testingdashboard.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreLevel1 extends Command {
  frc.robot.commands.WoS.Level1 m_WoSLevel1;
  frc.robot.commands.elevator.Level1 m_ElevatorLevel1;
  
  enum State {
    kInit,
  };

  State m_state = State.kInit;

  /** Creates a new ScoreLevel1. */
  public ScoreLevel1() {
    super(WoS.getInstance(), "Scoring Commands", "ScoreLevel1");
    // Use addRequirements() here to declare subsystem dependencies.
    m_WoSLevel1 = new frc.robot.commands.WoS.Level1();
    m_ElevatorLevel1 = new frc.robot.commands.elevator.Level1();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_state) {
      case kInit:
        m_WoSLevel1.schedule();
        m_ElevatorLevel1.schedule();
        m_state = State.kInit;
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
