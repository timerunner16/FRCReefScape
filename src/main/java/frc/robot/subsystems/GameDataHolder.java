// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.atomic.DoubleAccumulator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.testingdashboard.TDSendable;

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

  private Field2d m_stateField;

  private int m_displayLevel;
  private TDNumber m_TDdisplayLevel;

  /** Creates a new GameDataHolder. */
  private GameDataHolder() {
    super("GameDataHolder");
    ResetReef();

    m_stateField = new Field2d();
    m_stateField.setRobotPose(-10, 0, Rotation2d.kZero);
    new TDSendable(this, "Display", "Reef State", m_stateField);
    m_TDdisplayLevel = new TDNumber(this, "Display", "Current Level", 1);
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

  public void SetDisplayLevel(int level) {
    m_TDdisplayLevel.set(level);
  }

  @Override
  public void periodic() {
    m_displayLevel = (int)Math.round(m_TDdisplayLevel.get());
    if (m_displayLevel < 1) m_displayLevel = 1;
    if (m_displayLevel > 4) m_displayLevel = 4;
    for (int i = 0; i < 12; i++) {
      if (!GetState((int)m_displayLevel, i).filled) continue;
      double a = Math.toRadians(((i/2)/6.0)*360.0-90.0);
      Translation2d trans = new Translation2d(3,0)
        .rotateBy(new Rotation2d(a))
        .plus(new Translation2d(0,i%2==0?-2./3.:2./3.).rotateBy(new Rotation2d(a)));
      trans = new Translation2d(trans.getX()+6, -trans.getY()+6);
      Pose2d pose = new Pose2d(
        trans,
        new Rotation2d(Math.PI-a)
      );
      m_stateField.getObject(String.valueOf(i)).setPose(pose);
    }
  }
}
