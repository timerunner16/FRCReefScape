// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.testingdashboard.Command;
import frc.robot.utils.FieldUtils;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.WoS.Consume;
import frc.robot.commands.WoS.Expel;
import frc.robot.commands.drive.AlignToClosestReefLeft;
import frc.robot.commands.drive.AlignToClosestReefRight;
import frc.robot.commands.drive.AlignToLeftCoral;
import frc.robot.commands.drive.AlignToRightCoral;
import frc.robot.commands.drive.PathFindToPose;
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
  PathFindToPose m_driveToPose;
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

  boolean m_finishedDrive = false;
  boolean m_finishedElevator = false;

  Pose2d m_poseAtMoveStart;

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
    m_driveToPose = new PathFindToPose(this::GetTargetPose);

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
    // Move from the current position (probably start line or coral station) to the closest available reef state according to score strategy
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
          if (m_targetState != null && m_targetPose != null) break;
        }
        
        if (m_targetState == null || m_targetPose == null) {
          System.out.println("Finished MOVING_TO_REEF (no open reef state); cancelling");
          m_consume.cancel();
          m_driveToPose.cancel();
          m_autoState = AutoState.ALIGNING_TO_REEF;
          m_finishedElevator = false;
          m_finishedDrive = false;
          return;
        }
        
        m_gameDataHolder.SetDisplayLevel(m_targetState.level);

        m_consume.schedule();
        m_driveToPose.schedule();

        if ((Math.abs(m_drive.getMeasuredSpeeds().vxMetersPerSecond)+Math.abs(m_drive.getMeasuredSpeeds().vyMetersPerSecond)) < 0.05 &&
            m_drive.getPose().getTranslation().getDistance(m_targetPose.getTranslation()) < 0.05) {
          System.out.println("Finished MOVING_TO_REEF; advancing to ALIGNING_TO_REEF");
          m_consume.cancel();
          m_driveToPose.cancel();
          m_autoState = AutoState.ALIGNING_TO_REEF;
          m_finishedElevator = false;
          m_finishedDrive = false;
          
          m_poseAtMoveStart = m_drive.getPose();
        }

        break;
      }

      // Move more accurately to the correct position at the reef structre
      case ALIGNING_TO_REEF: {
        switch (m_targetState.level) {
          case 1: {
            m_level1.schedule();
            if (m_level1.isFinished()) m_finishedElevator = true;
            break;
          }
          case 2: {
            m_level2.schedule();
            if (m_level2.isFinished()) m_finishedElevator = true;
            break;
          }
          case 3: {
            m_level3.schedule();
            if (m_level3.isFinished()) m_finishedElevator = true;
            break;
          }
          case 4: {
            m_level4.schedule();
            if (m_level4.isFinished()) m_finishedElevator = true;
            break;
          }
        }
        if (m_targetState.position%2==0) {
          m_alignToClosestReefLeft.schedule();
          if (m_alignToClosestReefLeft.isFinished()) m_finishedDrive = true;
        } else {
          m_alignToClosestReefRight.schedule();
          if (m_alignToClosestReefRight.isFinished()) m_finishedDrive = true;
        }

        if (m_finishedDrive && m_finishedElevator) {
          System.out.println("Finished ALIGNING_TO_REEF; advancing to SCORING");
          m_autoState = AutoState.SCORING;

          m_poseAtMoveStart = m_drive.getPose();
        }

        break;
      }

      // Expel at the current state for a short time
      case SCORING: {
        m_timer.start();
        
        m_expel.schedule();

        if (m_timer.get() > 0.5) {
          System.out.println("Finished SCORING; advancing to MOVING_TO_CORAL");
          m_gameDataHolder.SetState(m_targetState.level, m_targetState.position, true);

          m_expel.cancel();

          m_timer.stop();
          m_timer.reset();

          m_autoState = AutoState.MOVING_TO_CORAL;

          m_poseAtMoveStart = m_drive.getPose();
        }
        break;
      }

      // Move from the reef structure to the coral station; inaccurate movement
      case MOVING_TO_CORAL: {
        double xOffsetBound = 14.0;
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
            if(alliance.get() == DriverStation.Alliance.Red){
                xOffsetBound = 14.0;
            } else if(alliance.get() == DriverStation.Alliance.Blue) {
                xOffsetBound = 3.5;
            }
        }
        CoralStationSide side;
        CoralStationOffset offset;
        if (m_poseAtMoveStart.getY() < 4.0) {
          side = CoralStationSide.kLeft;
          if (m_poseAtMoveStart.getX() < xOffsetBound) offset = CoralStationOffset.kRight;
          else offset = CoralStationOffset.kLeft;
        } else {
          side = CoralStationSide.kRight;
          if (m_poseAtMoveStart.getX() < xOffsetBound) offset = CoralStationOffset.kLeft;
          else offset = CoralStationOffset.kRight;
        }
        m_targetPose = m_fieldUtils.getCoralStationPose(side, offset);
        m_driveToPose.schedule();

        if ((Math.abs(m_drive.getMeasuredSpeeds().vxMetersPerSecond)+Math.abs(m_drive.getMeasuredSpeeds().vyMetersPerSecond)) < 0.05 &&
            m_drive.getPose().getTranslation().getDistance(m_targetPose.getTranslation()) < 0.05) {
          System.out.println("Finished MOVING_TO_CORAL; advancing to ALIGNING_TO_CORAL");
          m_driveToPose.cancel();
          m_autoState = AutoState.ALIGNING_TO_CORAL;
          m_timer.reset();
          m_timer.start();
          
          m_poseAtMoveStart = m_drive.getPose();
        }
        break;
      } 
      
      // Move more accurately to the correct position at the coral station while intaking until full
      case ALIGNING_TO_CORAL: {
        //m_alignToLeftCoral.schedule();
        m_feedingTime.schedule();

        if (m_feedingTime.isFinished() || (m_timer.get() > 1.0 && RobotBase.isSimulation())) {
          System.out.println("Finished ALIGNING_TO_CORAL; cycle complete; advancing to MOVING_TO_REEF");
          m_alignToLeftCoral.cancel();
          m_feedingTime.cancel();
          m_autoState = AutoState.MOVING_TO_REEF;
          m_timer.stop();
          m_timer.reset();
          
          m_poseAtMoveStart = m_drive.getPose();
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
