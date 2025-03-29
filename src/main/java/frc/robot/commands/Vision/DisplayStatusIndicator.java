// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.Lights.SolidLights;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Lights.LightSection;
import frc.robot.utils.vision.VisionEstimationResult;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DisplayStatusIndicator extends Command {
  SolidLights m_defaultIndicator;
  SolidLights m_reefIndicator;
  Vision m_vision;

  /** Creates a new DisplayReefCameraStatus. */
  public DisplayStatusIndicator() {
    addRequirements(Vision.getInstance());
    m_vision = Vision.getInstance();
    m_defaultIndicator = new SolidLights(Constants.Color.blue, LightSection.STATUS);
    m_reefIndicator = new SolidLights(Constants.Color.yellow, LightSection.STATUS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<VisionEstimationResult> reefresult = m_vision.getLatestFromCamera(Constants.VisionConstants.kReefCameraName);
    if (reefresult.isPresent()) {
      m_reefIndicator.schedule();
      m_defaultIndicator.cancel();
    } else {
      m_defaultIndicator.schedule();
      m_reefIndicator.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_reefIndicator.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
