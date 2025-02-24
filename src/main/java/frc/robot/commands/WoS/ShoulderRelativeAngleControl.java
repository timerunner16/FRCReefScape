// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WoS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.WoS;
import frc.robot.testingdashboard.Command;;

public class ShoulderRelativeAngleControl extends Command {
  WoS m_WoS;
  XboxController m_operatorController;

  /** Creates a new PivotRelativeAngleControl. */
  public ShoulderRelativeAngleControl() {
    super(WoS.getInstance(), "Basic", "ShoulderRelativeAngleControl");
    m_WoS = WoS.getInstance();
    m_operatorController = OI.getInstance().getOperatorXboxController();
    
    addRequirements(m_WoS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_WoS.getShoulderAngle();
    // upward input on joystick will move Barrel Pivot upwards and vice versa
    double input = -MathUtil.applyDeadband(m_operatorController.getLeftY(), Constants.WoSConstants.kShoulderDeadband);

    angle += input * Constants.WoSConstants.SHOULDER_ANGLE_INCREMENT_DEGREES;

    m_WoS.setTargetShoulderAngle(angle);
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
