// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.testingdashboard.ParallelCommandGroup;
import frc.robot.commands.WoS.Consume;
import frc.robot.commands.elevator.SetElevatorLevel;
import frc.robot.commands.funnel.Implode;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedingTime extends ParallelCommandGroup {
  /** Creates a new Level4Score. */
  public FeedingTime() {
    super(Elevator.getInstance(), "Levels", "FeedingTime");
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetElevatorLevel(0), new Implode(), new Consume());
  }
}
