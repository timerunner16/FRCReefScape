// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.OI;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.testingdashboard.Command;
import frc.robot.utils.FieldUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestTargetDrive extends Command {
  TargetDrive m_targetDriveCommand;

  /** Creates a new TestTargetDrive. */
  public TestTargetDrive() {
    super(AlgaeIntake.getInstance(), "", "TestTargetDrive");
    m_targetDriveCommand = new TargetDrive(()->{return FieldUtils.getInstance().getTagPose(
        FieldUtils.getInstance().getAllianceAprilTags().middleFrontReef).toPose2d();}, OI.getInstance().getDriveInputs());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetDriveCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_targetDriveCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
