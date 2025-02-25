// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WoS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.WoS;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WosFunnelTest extends Command {
  WoS m_wos;
  Funnel m_funnel;
  OI m_oi;

  /** Creates a new WosFunnelTest. */
  public WosFunnelTest() {
    m_wos = WoS.getInstance();
    m_funnel = Funnel.getInstance();
    m_oi = OI.getInstance();

    addRequirements(m_wos, m_funnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double wosPowerRatio = (Constants.WoSConstants.kWoSRPMSurfaceSpeedRatio/Constants.FunnelConstants.kFunnelRPMSurfaceSpeedRatio);
    double speed =  MathUtil.applyDeadband(m_oi.getOperatorXboxController().getLeftY(), Constants.OIConstants.kDriveDeadband);

    m_wos.spinIn(speed/wosPowerRatio);
    m_funnel.spinIn(speed);
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
