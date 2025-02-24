// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WoS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WoS;

import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Level1 extends Command {
  /** Creates a new SetLevel1. */
  WoS m_WoS;

  private boolean m_isFinished = false;

  public Level1() {
    m_WoS = WoS.getInstance();
    addRequirements(m_WoS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_WoS.setTargetLevel(1);

    if (MathUtil.isNear(Constants.WoSConstants.kWoSShoulderLevels[1], m_WoS.getShoulderAngle(), Math.PI/180.0)) {
       m_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_WoS.noSpin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
