// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.testingdashboard.SubsystemBase;

public class GameDataHolder extends SubsystemBase {
  public class ReefState {
    public int level;
    public int position;
    public boolean filled;

    ReefState(int level, int position, boolean filled) {
      this.level = level;
      this.position = position;
      this.filled = filled;
    }

    @Override
    public String toString() {
        return "[Level: " + level + " Position: " + position + " Filled: " + filled + "]";
    }
  }

  private ReefState[][] m_reefState = new ReefState[4][12];
  private static GameDataHolder m_gameDataHolder;

  /** Creates a new GameDataHolder. */
  private GameDataHolder() {
    super("GameDataHolder");
    ResetReef();
  }

  public static GameDataHolder getInstance() {
    if (m_gameDataHolder == null) {
      m_gameDataHolder = new GameDataHolder();
    }
    return m_gameDataHolder;
  }

  public void SetState(int level, int position, boolean state) {
    m_reefState[level-1][position] = new ReefState(level, position, state);
  }

  public ReefState GetState(int level, int position) {
    return m_reefState[level-1][position];
  }

  public void ResetReef() {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 12; j++) {
        m_reefState[i][j] = new ReefState(i+1, j, false);
      }
    }
  }

  @Override
  public void periodic() {}
}
