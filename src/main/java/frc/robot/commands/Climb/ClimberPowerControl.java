// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.testingdashboard.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberPowerControl extends Command {
  Climber m_climber;
  XboxController m_operatorController;
  /** Creates a new ClimberPowerControl. */
  public ClimberPowerControl() {
    super(Climber.getInstance(), "Climber", "Climber Power Control");
    m_climber = Climber.getInstance();
    m_operatorController = OI.getInstance().getOperatorXboxController();
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -MathUtil.applyDeadband(m_operatorController.getRightY(), Constants.OIConstants.kDriveDeadband);
    m_climber.winchIn(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
