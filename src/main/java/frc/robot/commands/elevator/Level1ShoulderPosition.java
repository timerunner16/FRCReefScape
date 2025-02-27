// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import frc.robot.testingdashboard.Command;
import frc.robot.subsystems.Elevator;

import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Level1ShoulderPosition extends Command {
  /** Creates a new SetLevel1. */
  Elevator m_elevator;

  private boolean m_isFinished = false;

  public Level1ShoulderPosition() {
    super(Elevator.getInstance(),"Elevator","Level1ShoulderPosition");
    m_elevator = Elevator.getInstance();
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setShoulderTargetAngle(1);

    if (MathUtil.isNear(Constants.ElevatorConstants.kShoulderLevels[1], m_elevator.getShoulderAngle(), Math.PI/180.0)) {
       m_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stopShoulder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
