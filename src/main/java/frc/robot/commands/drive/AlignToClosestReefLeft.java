// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.Command;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.FieldUtils.ReefFaceOffset;
import frc.robot.utils.TargetPose;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToClosestReefLeft extends Command {
  AdjustToReef m_adjustToReef;

  /** Creates a new AlignToClosestReefLeft. */
  public AlignToClosestReefLeft() {
    super(Drive.getInstance(), "Auto Commands", "AlignToClosestReefLeft");

    m_adjustToReef = new AdjustToReef(this::reefLeftPoseSupplier);
    addRequirements(Drive.getInstance());
  }

  private TargetPose reefLeftPoseSupplier() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return new TargetPose(FieldUtils.getInstance().getClosestReefScoringPosition(
      Drive.getInstance().getPose(), ReefFaceOffset.kLeft, alliance),
      true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_adjustToReef.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_adjustToReef.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_adjustToReef.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_adjustToReef.isFinished();
  }
}
