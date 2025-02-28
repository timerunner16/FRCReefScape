// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.testingdashboard.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShoulderPowerControl extends Command {
  Elevator m_elevator;
  XboxController m_operatorController;
  /** Creates a new ElevatorManualPowerControl. */
  public ShoulderPowerControl() {
    super(Elevator.getInstance(), "Shoulder", "Shoulder Power Control");
    m_elevator = Elevator.getInstance();
    m_operatorController = OI.getInstance().getOperatorXboxController();
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -MathUtil.applyDeadband(m_operatorController.getRightY(), Constants.OIConstants.kDriveDeadband);
    m_elevator.spinShoulder(speed);
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
