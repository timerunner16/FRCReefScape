// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WoS;

import frc.robot.Constants;
import frc.robot.commands.Lights.BlinkLights;
import frc.robot.commands.Lights.SolidLights;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.subsystems.WoS;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Consume extends Command {
  WoS m_WoS;

  TDNumber m_RPM;
  TDNumber m_enablePID;

  TDNumber m_WoSSpeed;

  BlinkLights m_blinkLights;
  SolidLights m_solidLights;

  /** Creates a new Consume. */
  public Consume() {
    super(WoS.getInstance(),"WoS","Consume");
    m_WoS = WoS.getInstance();

    m_RPM = new TDNumber(m_WoS, "WoS Speed (RPM)", "RPM", Constants.WoSConstants.kWoSSpeedRPM);
    m_enablePID = new TDNumber(m_WoS, "WoS Speed (RPM)", "Enable PID w 1");

    m_WoSSpeed = new TDNumber(m_WoS, "WoS Speed (Power)", "Speed", Constants.WoSConstants.kWoSSpeed);

    addRequirements(m_WoS);

    m_blinkLights = new BlinkLights(50);
    m_solidLights = new SolidLights(50);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_enablePID.get() == 1) {
      m_WoS.setSpeeds(m_RPM.get(), false);
    }
    else {
      m_WoS.spinIn(m_WoSSpeed.get());
    }

    if (m_WoS.getCoralDetected()) {
      m_blinkLights.cancel();
      m_solidLights.schedule();
    } else {
      m_solidLights.cancel();
      m_blinkLights.schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_WoS.noSpin(0);
    m_blinkLights.cancel();
      m_solidLights.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
