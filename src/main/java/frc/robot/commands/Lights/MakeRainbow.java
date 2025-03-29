// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lights;

import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightSection;
import frc.robot.testingdashboard.Command;

public class MakeRainbow extends Command {
  Lights m_lights;
  LightSection m_section;

  /** Creates a new MakeRainbow. */
  public MakeRainbow() {
    this(LightSection.ALL);
  }
  
  public MakeRainbow(LightSection section) {
    super(Lights.getInstance(), "Basic", "MakeRainbow");
    m_lights = Lights.getInstance();
    addRequirements(m_lights);

    m_section = section;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lights.makeRainbow(m_section);
    m_lights.setData();
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
