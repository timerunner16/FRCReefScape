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
  private TargetPose m_currentTarget;

  TDNumber TDCurrentTargetX;
  TDNumber TDCurrentTargetY;
  TDNumber TDCurrentTargetAngle;

  BlinkLights m_blinkLights;

  /** Creates a new TargetDrive. */
  public DriveToPose(Supplier<TargetPose> targetSupplier) {
    super(Drive.getInstance(), "Drive", "DriveToPose");
    m_drive = Drive.getInstance();
    m_currentTarget = targetSupplier.get();

    TDCurrentTargetX = new TDNumber(m_drive, "Test Outputs", "Current Target X");
    TDCurrentTargetX.set(m_currentTarget.getPose().getX());
    TDCurrentTargetY = new TDNumber(m_drive, "Test Outputs", "Current Target Y");
    TDCurrentTargetY.set(m_currentTarget.getPose().getY());
    TDCurrentTargetAngle = new TDNumber(m_drive, "Test Outputs", "Current Target Angle");
    TDCurrentTargetAngle.set(m_currentTarget.getPose().getRotation().getDegrees());

    addRequirements(m_drive);

    m_blinkLights = new BlinkLights(Constants.Color.red);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_blinkLights.schedule();

    if (m_currentPathCommand == null || m_currentTarget == null)
    {
      m_currentPathCommand = AutoBuilder.pathfindToPose(m_currentTarget.getPose(),
                                                        new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, 
                                                        Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                                                        Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared),
                                                        0);
      m_currentPathCommand.initialize();
    }
    m_currentPathCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_blinkLights.cancel();
    
    if (m_currentPathCommand != null) {
      m_currentPathCommand.end(interrupted);
      m_currentPathCommand = null;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_currentPathCommand != null) {
      return m_currentPathCommand.isFinished();
    }
    return true;
  }
}