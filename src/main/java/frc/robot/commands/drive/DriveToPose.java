// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.commands.Lights.BlinkLights;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.utils.TargetPose;

public class DriveToPose extends Command {
  private Drive m_drive;
  private edu.wpi.first.wpilibj2.command.Command m_currentPathCommand;
  private Supplier<TargetPose> m_targetSupplier;

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
  
    addRequirements(m_drive);

    m_blinkLights = new BlinkLights(Constants.Color.red);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_blinkLights.schedule();
    TargetPose target = m_targetSupplier.get();
    if (m_currentPathCommand == null && target != null && target.getPose() != null)
    {
      TDCurrentTargetX.set(target.getPose().getX());
      TDCurrentTargetY.set(target.getPose().getY());
      TDCurrentTargetAngle.set(target.getPose().getRotation().getDegrees());
      Pose2d targetPose = target.getReversedApproach()? 
          new Pose2d(target.getPose().getX(), target.getPose().getY(), target.getPose().getRotation().plus(Rotation2d.k180deg)) 
          : target.getPose();
      m_currentPathCommand = AutoBuilder.pathfindToPose(targetPose,
                                                        new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, 
                                                        Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                                                        Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared),
                                                        0);
      m_currentPathCommand.schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_blinkLights.cancel();
    
    if (m_currentPathCommand != null) {
      m_currentPathCommand.cancel();
      m_currentPathCommand = null;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_currentPathCommand != null) {
      return m_currentPathCommand.isFinished();
    }
    return false;
  }
}