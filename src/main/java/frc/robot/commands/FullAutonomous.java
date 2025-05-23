// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.testingdashboard.Command;
import frc.robot.utils.FieldUtils;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.WoS.Consume;
import frc.robot.commands.WoS.Expel;
import frc.robot.commands.drive.AlignToClosestReefLeft;
import frc.robot.commands.drive.AlignToClosestReefRight;
import frc.robot.commands.drive.AlignToLeftCoral;
import frc.robot.commands.drive.AlignToRightCoral;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.GameDataHolder;
import frc.robot.subsystems.GameDataHolder.ReefState;
import frc.robot.utils.TargetPose;
import frc.robot.utils.FieldUtils.CoralStationOffset;
import frc.robot.utils.FieldUtils.CoralStationSide;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FullAutonomous extends Command {
  public enum ScoreStrategy {
    SCORE,
    RANKING_POINT
  };

  enum AutoState {
    MOVING_TO_REEF,
    ALIGNING_TO_REEF,
    SCORING,
    MOVING_TO_CORAL,
    ALIGNING_TO_CORAL
  };

  Timer m_timer;
  AutoState m_autoState;
  FieldUtils m_fieldUtils;
  GameDataHolder m_gameDataHolder;
  Drive m_drive;

  ScoreStrategy m_scoreStrategy;
  ReefState m_targetState;
  Pose2d m_targetPose;
  DriveToPose m_driveToPose;
  Consume m_consume;

  AlignToClosestReefLeft m_alignToClosestReefLeft;
  AlignToClosestReefRight m_alignToClosestReefRight;

  Level1 m_level1;
  Level2 m_level2;
  Level3 m_level3;
  Level4 m_level4;
  Expel m_expel;

  AlignToLeftCoral m_alignToLeftCoral;
  AlignToRightCoral m_alignToRightCoral;

  FeedingTime m_feedingTime;

  /** Creates a new FullAutonomous. */
  public FullAutonomous() {
    this(ScoreStrategy.SCORE);
  }

  public FullAutonomous(ScoreStrategy scoreStrategy) {
    super(Drive.getInstance(), "Auto Commands", "FullAutonomous");

    m_timer = new Timer();
    m_timer.stop();
    m_timer.reset();

    m_autoState = AutoState.MOVING_TO_REEF;
    m_fieldUtils = FieldUtils.getInstance();
    m_gameDataHolder = GameDataHolder.getInstance();
    m_drive = Drive.getInstance();
    m_targetPose = null;
    m_driveToPose = new DriveToPose(this::GetTargetPose);

    m_scoreStrategy = scoreStrategy;
    m_targetState = null;
    m_consume = new Consume();

    m_alignToClosestReefLeft = new AlignToClosestReefLeft();
    m_alignToClosestReefRight = new AlignToClosestReefRight();
    m_level1 = new Level1();
    m_level2 = new Level2();
    m_level3 = new Level3();
    m_level4 = new Level4();

    m_expel = new Expel();

    m_alignToLeftCoral = new AlignToLeftCoral();
    m_alignToRightCoral = new AlignToRightCoral();

    m_feedingTime = new FeedingTime();
  }

  TargetPose GetTargetPose() {
    return new TargetPose(m_targetPose, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * GENERAL PLAN:
     * find optimal coral position based on score strategy and game state
     * go to reef face
     * align to closest reef face and move elevator
     * score
     * add score to reef structure
     * go to coral station
     * align to coral station while intaking
     * repeat
     */
    switch (m_autoState) {
      case MOVING_TO_REEF: {
        int level;
        int position;

        ArrayList<ArrayList<ReefState>> open = new ArrayList<ArrayList<ReefState>>(4);
        for (int i = 0; i < 4; i++) {open.add(new ArrayList<ReefState>());}

        for (level = 0; level < 4; level++) {
          for (position = 0; position < 12; position++) {
            ReefState state = GameDataHolder.getInstance().GetState(level+1, position);
            if (!state.filled) open.get(level).add(state);
          }
        }

        m_targetState = null;
        m_targetPose = null;
        double distance = 100.0;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        for (int i = 3; i>=0; i--) {
          if (m_scoreStrategy == ScoreStrategy.RANKING_POINT && 12-open.get(i).size() >= 5) continue;

          for (int j = 0; j < open.get(i).size(); j++) {
            ReefState reefState = open.get(i).get(j);
            Pose2d pose = m_fieldUtils.getPoseFromScorePosition(reefState, alliance);
            double new_distance = pose.getTranslation().getDistance(m_drive.getPose().getTranslation());
            if (new_distance < distance) {
              distance = new_distance;
              m_targetState = reefState;
              m_targetPose = pose;
            }
          }
        }

        m_consume.schedule();
        m_driveToPose.schedule();

        if (m_driveToPose.isFinished()) {
          m_consume.cancel();
          m_driveToPose.cancel();
          m_autoState = AutoState.ALIGNING_TO_REEF;
        }

        break;
      }

      case ALIGNING_TO_REEF: {
        boolean finished_drive = false;
        boolean finished_elevator = false;
        switch (m_targetState.level) {
          case 1: {
            m_level1.schedule();
            if (m_level1.isFinished()) finished_elevator = true;
            break;
          }
          case 2: {
            m_level2.schedule();
            if (m_level2.isFinished()) finished_elevator = true;
            break;
          }
          case 3: {
            m_level3.schedule();
            if (m_level3.isFinished()) finished_elevator = true;
            break;
          }
          case 4: {
            m_level4.schedule();
            if (m_level4.isFinished()) finished_elevator = true;
            break;
          }
        }
        if (m_targetState.position%2==0) {
          m_alignToClosestReefLeft.schedule();
          if (m_alignToClosestReefLeft.isFinished()) finished_drive = true;
        } else {
          m_alignToClosestReefRight.schedule();
          if (m_alignToClosestReefRight.isFinished()) finished_drive = true;
        }

        if (finished_drive && finished_elevator) {
          m_autoState = AutoState.SCORING;
        }

        break;
      }

      case SCORING: {
        m_timer.start();
        
        m_expel.schedule();

        if (m_timer.get() > 0.5) {
          m_gameDataHolder.SetState(m_targetState.level, m_targetState.position, true);

          m_expel.cancel();

          m_timer.stop();
          m_timer.reset();

          m_autoState = AutoState.MOVING_TO_CORAL;
        }
        break;
      }

      case MOVING_TO_CORAL: {
        m_targetPose = m_fieldUtils.getCoralStationPose(CoralStationSide.kLeft, CoralStationOffset.kRight);
        m_driveToPose.schedule();

        if (m_driveToPose.isFinished()) {
          m_driveToPose.cancel();
          m_autoState = AutoState.ALIGNING_TO_CORAL;
        }
        break;
      }
      
      case ALIGNING_TO_CORAL: {
        m_alignToLeftCoral.schedule();
        m_feedingTime.schedule();

        if (m_feedingTime.isFinished()) {
          m_alignToLeftCoral.cancel();
          m_feedingTime.cancel();
          m_autoState = AutoState.MOVING_TO_REEF;
        }
        break;
      }

      default: {
        m_autoState = AutoState.MOVING_TO_REEF;
        break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveToPose.cancel();
    m_consume.cancel();
    m_alignToClosestReefLeft.cancel();
    m_alignToClosestReefRight.cancel();
    m_level1.cancel();
    m_level2.cancel();
    m_level3.cancel();
    m_level4.cancel();
    m_expel.cancel();
    m_alignToLeftCoral.cancel();
    m_alignToRightCoral.cancel();
    m_feedingTime.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
