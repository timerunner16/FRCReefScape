// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import frc.robot.testingdashboard.Command;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.commands.Lights.BlinkLights;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lights.LightSection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorLevel extends Command {
  /** Creates a new Level1. */

  Elevator m_Elevator;
  boolean m_elevatorSet;
  boolean m_shoulderSet;
  boolean m_finished;
  int m_level;

  BlinkLights m_blinkLights;

  public SetElevatorLevel(int level) {
    super(Elevator.getInstance(), "Elevator", "Level1");
    m_Elevator = Elevator.getInstance();
    addRequirements(m_Elevator);
    m_level = level;

    m_blinkLights = new BlinkLights(Constants.Color.red, LightSection.ACTIVE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSet = false;
    m_shoulderSet = false;
    m_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_blinkLights.schedule();
    if (!m_elevatorSet) {
      m_Elevator.setElevatorTargetLevel(m_level);
      m_elevatorSet = true;
    }
    if(!m_shoulderSet && (m_level <= 2 || m_Elevator.getElevatorAngle() > Constants.ElevatorConstants.kElevatorDelayHeight)) {
      m_Elevator.setShoulderTargetLevel(m_level);
      m_shoulderSet = true;
    }

    if (m_Elevator.inGoalPosition()) {
      m_finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_blinkLights.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished || RobotBase.isSimulation();
  }
}
