// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;


import frc.robot.Constants;

public class MAXSwerveModule {
  private final SparkFlex m_drivingSparkFlex;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private double m_lastAngle = 0;
  private double m_lastSpeed = 0;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkFlex = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    /* Configure the PID, FFF and other properties of the turning motors. Most of these come from 
     * Cnstants.
    */
    SparkMaxConfig m_turningConfig = new SparkMaxConfig();
    m_turningConfig.absoluteEncoder.inverted(Constants.kTurningEncoderInverted);
    m_turningConfig.idleMode(Constants.kTurningMotorIdleMode);
    m_turningConfig.smartCurrentLimit(Constants.kTurningMotorCurrentLimit);
    m_turningConfig.absoluteEncoder.positionConversionFactor(Constants.kTurningEncoderPositionFactor);
    m_turningConfig.absoluteEncoder.velocityConversionFactor(Constants.kTurningEncoderVelocityFactor);
    m_turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    m_turningConfig.closedLoop.p(Constants.kTurningP);
    m_turningConfig.closedLoop.i(Constants.kTurningI);
    m_turningConfig.closedLoop.d(Constants.kTurningD);
    m_turningConfig.closedLoop.velocityFF(Constants.kTurningFF);
    m_turningConfig.closedLoop.positionWrappingEnabled(true);
    m_turningConfig.closedLoop.positionWrappingMinInput(Constants.kTurningEncoderPositionPIDMinInput);
    m_turningConfig.closedLoop.positionWrappingMinInput(Constants.kTurningEncoderPositionPIDMaxInput);
    m_turningConfig.closedLoop.outputRange(Constants.kTurningMinOutput, Constants.kTurningMaxOutput);
   
    m_turningSparkMax.configure(m_turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();

    /* Configure the PID, FFF and other properties of the driving motors. Most of these come from 
     * Cnstants.
    */
    SparkFlexConfig m_drivingConfig = new SparkFlexConfig();
    m_drivingConfig.idleMode(Constants.kDrivingMotorIdleMode);
    m_drivingConfig.inverted(false);
    m_drivingConfig.smartCurrentLimit(Constants.kDrivingMotorCurrentLimit);
    m_drivingConfig.encoder.positionConversionFactor(Constants.kDrivingEncoderPositionFactor);
    m_drivingConfig.encoder.velocityConversionFactor(Constants.kDrivingEncoderVelocityFactor);
    m_drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    m_drivingConfig.closedLoop.p(Constants.kDrivingP);
    m_drivingConfig.closedLoop.i(Constants.kDrivingI);
    m_drivingConfig.closedLoop.d(Constants.kDrivingD);
    m_drivingConfig.closedLoop.velocityFF(Constants.kDrivingFF);
    m_drivingConfig.closedLoop.outputRange(Constants.kDrivingMinOutput, Constants.kDrivingMaxOutput);
   
    m_drivingSparkFlex.configure(m_drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_drivingEncoder = m_drivingSparkFlex.getEncoder();
    m_drivingPIDController = m_drivingSparkFlex.getClosedLoopController();
   
    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }
  
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
    
    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    if (correctedDesiredState.speedMetersPerSecond != m_lastSpeed) {
      m_lastSpeed = correctedDesiredState.speedMetersPerSecond;
      m_drivingPIDController.setReference(correctedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    }
    if (correctedDesiredState.angle.getRadians() != m_lastAngle) {
      m_lastAngle = correctedDesiredState.angle.getRadians();
      m_turningPIDController.setReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);
    }

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public double getLastSpeed()
  {
    return m_lastSpeed;
  }

  public double getDriveOutputCurrent() {
    return m_drivingSparkFlex.getOutputCurrent();
  }
  public double getTurningOutputCurrent() {
    return m_turningSparkMax.getOutputCurrent();
  }
}