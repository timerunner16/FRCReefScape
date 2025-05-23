// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.testingdashboard.Command;
import frc.robot.utils.TargetPose;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AdjustToCoral extends Command {
  DriveToPose m_DriveToPose;
  SwerveDrivePoseEstimator m_poseEstimator;
  Supplier<TargetPose> m_targetSupplier;
  Drive m_drive;
  Vision m_vision;
  boolean m_gotVisionMeasurement;

  double m_shoulderLastTargetAngle;

  /** Creates a new AdjustToCoral. */
  public AdjustToCoral(Supplier<TargetPose> targetSupplier) {
    super(Drive.getInstance(), "Drive", "AdjustToCoral");
    m_drive = Drive.getInstance();
    m_vision = Vision.getInstance();
    
    m_poseEstimator = 
      new SwerveDrivePoseEstimator(
        Constants.DriveConstants.m_kinematics, 
        Rotation2d.fromDegrees(m_drive.getGyroAngle()), 
        m_drive.getModulePositions(), 
        m_drive.getPose());
    m_targetSupplier = targetSupplier;

    m_DriveToPose = new DriveToPose(m_targetSupplier, m_poseEstimator::getEstimatedPosition);
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var visionResult = m_vision.getLatestFromCamera(Constants.VisionConstants.kCoralCameraName);
    if(visionResult.isPresent())
    {
      m_poseEstimator.resetPosition(Rotation2d.fromDegrees(m_drive.getGyroAngle()), m_drive.getModulePositions(), visionResult.get().estimatedPose.toPose2d());
      m_gotVisionMeasurement = true;
    } else {
      m_poseEstimator.resetPosition(Rotation2d.fromDegrees(m_drive.getGyroAngle()), m_drive.getModulePositions(), m_drive.getPose());
      m_gotVisionMeasurement = false;
    }

    m_DriveToPose.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var visionResult = m_vision.getLatestFromCamera(Constants.VisionConstants.kCoralCameraName);
    if(!m_gotVisionMeasurement && visionResult.isPresent()) {
      m_poseEstimator.resetPosition(Rotation2d.fromDegrees(m_drive.getGyroAngle()), m_drive.getModulePositions(), visionResult.get().estimatedPose.toPose2d());
      m_gotVisionMeasurement = true;
    }

    m_poseEstimator.update(Rotation2d.fromDegrees(m_drive.getGyroAngle()), m_drive.getModulePositions());

    if(visionResult.isPresent()) {
      m_poseEstimator.addVisionMeasurement(
        visionResult.get().estimatedPose.toPose2d(),
        visionResult.get().timestamp,
        visionResult.get().stdDevs);
    }

    m_DriveToPose.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveToPose.end(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_DriveToPose.isFinished();
  }
}
