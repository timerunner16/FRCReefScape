// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import frc.robot.commands.Lights.BlinkLights;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.utils.TargetPose;

public class DriveToPose extends Command {
  private Drive m_drive;
  private Supplier<TargetPose> m_targetSupplier;
  private Pose2d m_targetPose;

  private ProfiledPIDController m_thetaController;
  private ProfiledPIDController m_driveXController;
  private ProfiledPIDController m_driveYController;

  TDNumber TDCurrentTargetX;
  TDNumber TDCurrentTargetY;
  TDNumber TDCurrentTargetAngle;

  BlinkLights m_blinkLights;

  /** Creates a new TargetDrive. */
  public DriveToPose(Supplier<TargetPose> targetSupplier) {
    super(Drive.getInstance(), "Drive", "DriveToPose");
    m_drive = Drive.getInstance();
    m_targetSupplier = targetSupplier;

    TDCurrentTargetX = new TDNumber(m_drive, "Test Outputs", "Current Target X");
    TDCurrentTargetY = new TDNumber(m_drive, "Test Outputs", "Current Target Y");
    TDCurrentTargetAngle = new TDNumber(m_drive, "Test Outputs", "Current Target Angle");

    m_thetaController = new ProfiledPIDController(5.0, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_driveXController = new ProfiledPIDController(5, 0, 0, new Constraints(4.8, 4.8));
    m_driveYController = new ProfiledPIDController(5, 0, 0, new Constraints(4.8, 4.8));
  
    addRequirements(m_drive);

    m_blinkLights = new BlinkLights(Constants.Color.red);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_blinkLights.schedule();
    TargetPose target = m_targetSupplier.get();
    if (target != null && target.getPose() != null)
    {
      TDCurrentTargetX.set(target.getPose().getX());
      TDCurrentTargetY.set(target.getPose().getY());
      TDCurrentTargetAngle.set(target.getPose().getRotation().getDegrees());
      m_targetPose = target.getReversedApproach()? 
          new Pose2d(target.getPose().getX(), target.getPose().getY(), target.getPose().getRotation().plus(Rotation2d.k180deg)) 
          : target.getPose();
      m_thetaController.reset(m_drive.getPose().getRotation().getRadians());
      m_driveXController.reset(m_drive.getPose().getX());
      m_driveYController.reset(m_drive.getPose().getY());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_targetPose != null)
    {
      double omega = m_thetaController.calculate(m_drive.getPose().getRotation().getRadians(), m_targetPose.getRotation().getRadians());
      double xVel = m_driveXController.calculate(m_drive.getPose().getX(), m_targetPose.getX());
      double yVel = m_driveYController.calculate(m_drive.getPose().getY(), m_targetPose.getY());

      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, omega,  m_drive.getPose().getRotation());
      m_drive.drive(speeds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_blinkLights.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_targetPose == null) ||
      (m_drive.getPose().getTranslation().getDistance(m_targetPose.getTranslation()) < Constants.AutoConstants.kTranslationTolerance) &&
      (Math.abs(m_drive.getPose().getRotation().minus(m_targetPose.getRotation()).getRadians()) < Constants.AutoConstants.kThetaTolerance);
  }
}