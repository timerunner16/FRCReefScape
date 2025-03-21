// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.ParallelDeadlineGroup;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BasicestAuto extends ParallelDeadlineGroup {
  /** Creates a new BasicestAuto. */
  public BasicestAuto() {
    super(Drive.getInstance(), "Auto Commands", "Basicest_Auto", new WaitCommand(1.5), new DriveForward());
  }
}
