// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.Command;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.FieldUtils.CoralStationOffset;
import frc.robot.utils.FieldUtils.CoralStationSide;
import frc.robot.utils.TargetPose;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToRightCoral extends Command {
  AdjustToCoral m_adjustToCoral;

  /** Creates a new AlignToClosestReefRight. */
  public AlignToRightCoral() {
    super(Drive.getInstance(), "Auto Commands", "AlignToRightCoral");

    m_adjustToCoral = new AdjustToCoral(this::coralPoseSupplier);
    addRequirements(Drive.getInstance());
  }

  private TargetPose coralPoseSupplier() {
    return new TargetPose(FieldUtils.getInstance().getCoralStationPose(CoralStationSide.kRight, CoralStationOffset.kLeft),
      false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_adjustToCoral.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_adjustToCoral.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_adjustToCoral.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_adjustToCoral.isFinished();
  }
}
