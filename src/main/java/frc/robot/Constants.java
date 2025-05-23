// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.TargetPose;
import frc.robot.utils.vision.VisionConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final String robotName = "robot1";

  public static final class Color {
    public static final int red = 0;
    public static final int orange = 15;
    public static final int yellow = 30;
    public static final int green = 60;
    public static final int blue = 125;
    public static final int purple = 145; // maybe?
  }
  
  public static final class AlgaeIntakeConstants {
    //AlgaeIntake constants
    public static final boolean kEnableAnglePIDTuning = false;
    public static final double kAngleP = 0;
    public static final double kAngleI = 0;
    public static final double kAngleD = 0;

    public static final double kAngleSpeed = 0.2;
    public static final double kAngleSpeedRPM = 30;

    public static final double kAngleLowerLimitDegrees = 0;
    public static final double kAngleUpperLimitDegrees = 0;

    public static final double kAngleScorePositionDegrees = 0;
    public static final double kAngleIntakePositionDegrees = 0;

    public static final double kAngleEncoderPositionFactor = (2 * Math.PI);

    public static final boolean kEnableRollerPIDTuning = false;
    public static final double kRollerP = 0;
    public static final double kRollerI = 0;
    public static final double kRollerD = 0;

    public static final double kRollerSpeed = 0.2;
    public static final double kRollerSpeedRPM = 30;

    public static final double DEGREES_PER_REVOLUTION = 360;
  }

  public static final class WoSConstants {
    //WoS constants
    public static final boolean kEnableWheelPIDTuning = false;
    public static final double kWoSP = 0;
    public static final double kWoSI = 0;
    public static final double kWoSD = 0;

    public static final double kWoSSpeed = 0.4;
    public static final double kWoSSpeedRPM = 30;

    public static final double kWoSRPMSurfaceSpeedRatio = 1.5;
  }

  public static final class ElevatorConstants {
    //Elevator Constants
    public static final boolean kEnableElevatorPIDTuning = false;
    public static final boolean kEnableElevatorClosedLoopControl = true;
    public static final double kElevatorP = 0.3;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kElevatorkS = 0.20;
    public static final double kElevatorkG = 0.07;
    public static final double kElevatorkV = 0.435;
    public static final double kElevatorkA = 0.0;
    public static final double kElevatorToleranceInches = 0.3;

    public static final double kElevatorMaxVelocity = 35;
    public static final double kElevatorMaxAcceleration = 35;
    public static final double kElevatorSpeed = 0.2;
    public static final double kElevatorSpeedRPM = 30;

    //Conversion from motor rotations to elevator linear inches
    public static final double kElevatorEncoderPositionFactor = (20.0/22.0*0.25); /* (planetary ratio) / (sprocket teeth) * (Inches per tooth) */

    public static final double DEGREES_PER_REVOLUTION = 360;
    public static final double kElevatorLowerLimitInches = 0;
    public static final double kElevatorUpperLimitInches = 26.5;

    public static final double[] kElevatorLevels = {
      0,
      0,
      1.6,
      7.5,
      kElevatorUpperLimitInches,
    };
    public static final double kElevatorLowAlgaeRemove = 1.5;
    public static final double kElevatorHighAlgaeRemove = 10.0;
    public static final double kElevatorDelayHeight = 4.0;

    public static final double kElevatorEatHeight = 0;
    public static final double kElevatorHeightIncrementInches = 0.25;

    public static final boolean kEnableShoulderPIDTuning = false;
    public static final boolean kEnableShoulderClosedLoopControl = true;
    public static final double kShoulderP = 0.005000;
    public static final double kShoulderI = 0;
    public static final double kShoulderD = 0;
    public static final double kShoulderkS = 0;
    public static final double kShoulderkG = -0.200000;
    public static final double kShoulderkV = 0;
    public static final double kShoulderToleranceDegrees = 2;

    public static final double kShoulderSpeed = 0.2;
    public static final double kShoulderSpeedRPM = 30;
    
    //ShoulderPositionFactor = (Degress/rev) * (Bottom sprocket Teeth) / (Planetary ratio) * (Top sprocket teeth)
    public static final double kShoulderEncoderPositionFactor = 360.0 * 18.0 / (25.0 * 40.0); 
    public static final double kShoulderMotorToShoulderRatio = 40/18.0;

    public static final double kShoulderLowerLimitDegrees = -5;
    public static final double kShoulderWEEEEE = 180;
    public static final double kShoulderUpperLimitDegrees = 290;

    public static final double[] kShoulderLevels = {
      0,
      0,
      228,
      204.5,
      200
    };
    public static final double kShoulderHighAlgaeRemove = 222;
    public static final double kShoulderLowAlgaeRemove = 236;

    public static final double kShoulderEatAngle = 0;
    public static final double SHOULDER_ANGLE_INCREMENT_DEGREES = 2;
    public static final double kShoulderDeadband = 0.05;
  }

  public static final class FunnelConstants {
    public static final boolean kEnableFunnelPIDTuning = false;
    public static final double kFunnelP = 0;
    public static final double kFunnelI = 0;
    public static final double kFunnelD = 0;

    public static final double kFunnelSpeed = 0.2;
    public static final double kFunnelSpeedRPM = 30;

    public static final double kFunnelRPMSurfaceSpeedRatio = 4.18;
  }

  public static final class LightsConstants {
    // Defines Lights constants
    public static final int LED_LENGTH = 49; // number of LEDs
    public static final double kBlinkDelay = 0.35;
    public static final int kNumFlashes = 10;
    public static final double kFlashDelayMinimum = 0.3;
    public static final double kFlashDelayMaximum = 0.5;
    public static final int kPositionSplitIndex = 10;
    public static final boolean kInversePolarity = true;
  }
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.92;
    public static final double kSlowDrive = 0.1;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
   // Locations for the swerve drive modules relative to the robot center.
    public static final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    public static final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    public static final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    public static final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
    // Creating my kinematics object using the module locations
    public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );
    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = Math.PI;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // Drive constants for AutoBuilder configuration
    // Distance from robot center to furthest module
    public static final double kBaseRadius = Units.inchesToMeters(RobotMap.R_BASE_RADIUS_INCHES);
    public static final Translation2d[] m_ModulePositions = new Translation2d[] { m_frontRightLocation, m_frontLeftLocation, m_backRightLocation, m_backLeftLocation };

    // SPARK MAX CAN IDs are in RobotMap

    public static final boolean kGyroReversed = false;
  }

  public static final class OIConstants {
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.92;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.92;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 10;
    public static final double kIThetaController = 0.1;
    public static final double kDThetaController = 0.05;
    public static final double kThetaTolerance = 0.02;//radians
    public static final double kTranslationTolerance = 0.025;//meters
    public static final int kReefFinishedPeriodics = 25;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  
    // AutoBuilder dynamic robot constriants
    public static final PIDConstants kPathFollowerTranslationPID = new PIDConstants(5.0, 0.0, 0.0); // Translation PID constants
    public static final PIDConstants kPathFollowerRotationPID = new PIDConstants(5.0, 0.0, 0.0); // Rotation PID constants    

    public static final double kPathFollowerMaxSpeed = Constants.kMaxSpeedMetersPerSecond; // Max module speed, in m/s
    public static final double kPathFollowerBaseRadius = DriveConstants.kBaseRadius; // Drive base radius in meters
    public static final double kPathFollowerMass = 52.1631; // 115 pounds
    public static final double kPathFollowerMomentOfInertia = 6.2; // Total guess. Rough estimate of l^2 + w^2 * m * 1/12
    public static final double kPathFollowerWheelCoeficientFriction = 1.2; // Total guess. pathplaner default

}

  public static final class VisionConstants {
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    
    // The standard deviations of our vision estimated poses, which affect correction rate
    // TODO: (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    
    // Maximum ambiguity accepted as a valid result from the vision systems
    public static final double kMaxValidAmbiguity = 0.2;
    public static final double kMaxZError = 0.75;
    public static final double kMaxRollError = 0.5;
    public static final double kMaxPitchError = 0.5;

    // TODO: These values are from Mania! Must be determined for new robot...
    public static final VisionConfig[] kManiaVisionSystems = {
        new VisionConfig("Arducam_OV9281_USB_Camera",
                         new Transform3d(new Translation3d(Units.inchesToMeters(15.5), Units.inchesToMeters(0.0), Units.inchesToMeters(21.5)), 
                                new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0))),
                         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                         PoseStrategy.LOWEST_AMBIGUITY),
        new VisionConfig("Arducam_OV2311_USB_Camera",
                         new Transform3d(new Translation3d(Units.inchesToMeters(15.75), Units.inchesToMeters(8.0), Units.inchesToMeters(8.0)), 
                new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0))),
                         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                         PoseStrategy.LOWEST_AMBIGUITY)
    };

    // Camera height from floor in inches
    public static final double kCameraHeight = 9;
    // Camera Width (Y) and Length (x) offsets in inches
    public static final double kCameraWidthOffset = 25.5/2;
    public static final double kCameraLengthOffset = 25.5/2;
    // Camera mount angle in degrees
    public static final double kCameraMountAngleYaw = 45;
    
    public static final String kFrontLeftCameraName = "Arducam_OV9782_D";
    public static final String kCoralCameraName = "Arducam_OV2311_A";
    public static final String kReefCameraName = "Arducam_OV9782_C";
    public static final String kRearRightCameraName = "Arducam_OV9782_B";
    public static final VisionConfig[] kTwigVisionSystems = {
      // Left rear camera, facing rear right corner
      new VisionConfig(kReefCameraName,
        new Transform3d(new Translation3d(
                  Units.inchesToMeters(-9.5)
                  , Units.inchesToMeters(0)
                  , Units.inchesToMeters(35)), 
        new Rotation3d(0, Units.degreesToRadians(55), Units.degreesToRadians(180))),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                PoseStrategy.LOWEST_AMBIGUITY),
      // Left front camera, facing front right corner
      new VisionConfig(kFrontLeftCameraName,
              new Transform3d(new Translation3d(
                        Units.inchesToMeters(Constants.VisionConstants.kCameraLengthOffset)
                        , Units.inchesToMeters(Constants.VisionConstants.kCameraWidthOffset)
                        , Units.inchesToMeters(Constants.VisionConstants.kCameraHeight)), 
              new Rotation3d(0, 0, Units.degreesToRadians(-1 * Constants.VisionConstants.kCameraMountAngleYaw))),
                       PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                       PoseStrategy.LOWEST_AMBIGUITY),
      // Right front camera, facing rear right corner
      new VisionConfig(kCoralCameraName,
              new Transform3d(new Translation3d(
                        Units.inchesToMeters(-3.5)
                        , Units.inchesToMeters(-10.5)
                        , Units.inchesToMeters(34.25)),
              new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(0))),
                       PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                       PoseStrategy.LOWEST_AMBIGUITY),
      // Right rear camera, facing rear left
      new VisionConfig(kRearRightCameraName,
              new Transform3d(new Translation3d(
                        Units.inchesToMeters(-1 * Constants.VisionConstants.kCameraLengthOffset)
                        , Units.inchesToMeters(-1 * Constants.VisionConstants.kCameraWidthOffset)
                        , Units.inchesToMeters(Constants.VisionConstants.kCameraHeight)), 
              new Rotation3d(0, 0, Units.degreesToRadians(90 + Constants.VisionConstants.kCameraMountAngleYaw))),
                       PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                       PoseStrategy.LOWEST_AMBIGUITY),
  };
}

  public static final class ClimberConstants {
    public static final double kWinchP = 0;
    public static final double kWinchI = 0;
    public static final double kWinchD = 0;
    public static final boolean kEnableWinchPIDTuning = false;

    public static final double kWinchSpeed = 1;
    public static final double kPositionConversionFactor = 1.0/100.0;
    public static final double kReleasedPosition = 5;//This is a wild guess, needs to be actually found
    public static final double kPositionTolerance = 0.05;
  }

  public static final class FieldLocationConstants {
    public static final double kMidfieldX = 8.75;
    public static final Pose2d kBlueReefCenter = new Pose2d(4.5, 4, Rotation2d.kZero);
    public static final Pose2d kRedReefCenter = new Pose2d(13, 4, Rotation2d.kZero);

    public static final Translation2d kReefLeftScoreTrans = new Translation2d((DriveConstants.kWheelBase/2)+Units.inchesToMeters(5.75), -0.2);
    public static final Translation2d kReefRightScoreTrans = new Translation2d((DriveConstants.kWheelBase/2)+Units.inchesToMeters(5.75), 0.2);//Should be the same but with -y

    public static final Translation2d kCoralLeftTrans = new Translation2d((DriveConstants.kWheelBase/2)+Units.inchesToMeters(5.75), -0.5);
    public static final Translation2d kCoralRightTrans = new Translation2d((DriveConstants.kWheelBase/2)+Units.inchesToMeters(5.75), 0.5);

    public static final TargetPose kRedCoralA1Pose = new TargetPose(new Pose2d(15.878, 0.773, new Rotation2d(Units.degreesToRadians(125))), true);
    public static final TargetPose kRedCoralA2Pose = new TargetPose(new Pose2d(16.858, 1.382, new Rotation2d(Units.degreesToRadians(125))), true);

  }

  //Driver control rate limits
  public static final double kMaxAccelerationMetersPerSecondSquared = 10;
  public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4*Math.PI;
  public static final double kMaxSpeedMetersPerSecond = 4.92;
  public static final double kMaxAngularSpeed = 2 * Math.PI;
  public static final double kWheelDiameterMeters = 0.0762;
  public static final double D_ANGLE_TOLERANCE_DEGREES = 2.5;
  public static final int kDrivingMotorPinionTeeth = 12;
  public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);

  public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians
    
    // Defines Neo Motor constant
    public static final double kVortexFreeSpeedRpm = 6784;

     // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;
    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = kVortexFreeSpeedRpm / 60;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
   
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
    / kDrivingMotorReduction; // meters per second

    public static final double kDrivingP = 0.02;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kCoast;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 80; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

    //Enables SysID Characterization Mode. !!Should be false during competitions. Can cause the Operator controller to be remapped!!
    public static final boolean kSysIdModeEnabled = false;
}