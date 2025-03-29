// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lights;

import frc.robot.testingdashboard.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightSection;

public class MoveLights extends Command {
  Lights m_lights;
  LightSection m_section;
  int m_hue;

  /** Creates a new MoveLights. */
  public MoveLights(int hue) {
    this(hue, LightSection.ALL);
  }

  public MoveLights(int hue, LightSection section) {
    super(Lights.getInstance(), "Basic", "MoveLights");
    m_lights = Lights.getInstance();

    addRequirements(m_lights);

    m_hue = hue;
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
    m_lights.moveLights(m_hue, m_section);
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
