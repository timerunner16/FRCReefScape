// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.TDNumber;

public class Elevator extends SubsystemBase {
  private static Elevator m_elevator;

  private final double periodTime = 0.02;

  TDNumber m_targetAngle;
  TDNumber m_elevatorEncoderValueRotations;
  TDNumber m_elevatorEncoderValueDegrees;
  TDNumber m_elevatorCurrentOutput;
  TDNumber m_TDelevatorP;
  TDNumber m_TDelevatorI;
  TDNumber m_TDelevatorD;
  double m_elevatorP = Constants.ElevatorConstants.kElevatorP;
  double m_elevatorI = Constants.ElevatorConstants.kElevatorI;
  double m_elevatorD = Constants.ElevatorConstants.kElevatorD;
  TDNumber m_encoderValueRotations;
  private double m_lastAngle = 0;

  SparkFlex m_leftSparkFlex;
  SparkFlex m_rightSparkFlex;

  SparkFlexConfig m_leftSparkFlexConfig;

  TDNumber m_leftCurrentOutput;
  TDNumber m_rightCurrentOutput;

  SparkClosedLoopController m_closedLoopController;
  SparkAbsoluteEncoder m_absoluteEncoder;

  ElevatorFeedforward m_elevatorFeedForwardController;
  TrapezoidProfile m_profile;
  TrapezoidProfile.State m_state;
  TrapezoidProfile.State m_setpoint;

  /** Creates a new Elevator. */
  private Elevator() {
    super("Elevator");

    if (RobotMap.E_ENABLED) {
      m_leftSparkFlex = new SparkFlex(RobotMap.E_LEFTMOTOR, MotorType.kBrushless);
      m_rightSparkFlex = new SparkFlex(RobotMap.E_RIGHTMOTOR, MotorType.kBrushless);

      m_leftSparkFlexConfig = new SparkFlexConfig();
      SparkFlexConfig rightElevatorSparkFlexConfig = new SparkFlexConfig();

      rightElevatorSparkFlexConfig.follow(m_leftSparkFlex, true);

      m_TDelevatorP = new TDNumber(this, "Elevator PID", "P", Constants.ElevatorConstants.kElevatorP);
      m_TDelevatorI = new TDNumber(this, "Elevator PID", "I", Constants.ElevatorConstants.kElevatorI);
      m_TDelevatorD = new TDNumber(this, "Elevator PID", "D", Constants.ElevatorConstants.kElevatorD);

      m_leftSparkFlexConfig.closedLoop.pid(Constants.ElevatorConstants.kElevatorP, Constants.ElevatorConstants.kElevatorI,
          Constants.ElevatorConstants.kElevatorD);
      m_leftSparkFlexConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      m_leftSparkFlexConfig.closedLoop.positionWrappingEnabled(false);

      m_leftSparkFlexConfig.absoluteEncoder.positionConversionFactor(Constants.ElevatorConstants.kElevatorEncoderPositionFactor);
      m_leftSparkFlexConfig.absoluteEncoder.inverted(false);

      m_leftSparkFlex.configure(m_leftSparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_closedLoopController = m_leftSparkFlex.getClosedLoopController();
      m_absoluteEncoder = m_leftSparkFlex.getAbsoluteEncoder();

      m_elevatorFeedForwardController = new ElevatorFeedforward(Constants.ElevatorConstants.kElevatorkS, Constants.ElevatorConstants.kElevatorkG, Constants.ElevatorConstants.kElevatorkV);
      m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        Constants.ElevatorConstants.kElevatorMaxVelocity,
        Constants.ElevatorConstants.kElevatorMaxAcceleration
      ));
      m_setpoint = new TrapezoidProfile.State(m_absoluteEncoder.getPosition(), 0.0);
      m_state = new TrapezoidProfile.State(m_absoluteEncoder.getPosition(), 0.0);

      m_targetAngle = new TDNumber(this, "Elevator Encoder Values", "Target Angle", getAngle());
      m_elevatorEncoderValueRotations = new TDNumber(this, "Elevator Encoder Values", "Rotations", getAngle() / Constants.ElevatorConstants.kElevatorEncoderPositionFactor);
      m_elevatorEncoderValueDegrees = new TDNumber(this, "Elevator Encoder Values", "Angle (degrees)", getAngle());
      m_leftCurrentOutput = new TDNumber(this, "Current", "Left Elevator Output", m_leftSparkFlex.getOutputCurrent());
      m_rightCurrentOutput = new TDNumber(this, "Current", "Right Elevator Output", m_rightSparkFlex.getOutputCurrent());
    }
  }

  public static Elevator getInstance() {
    if (m_elevator == null) {
      m_elevator = new Elevator();
    }
    return m_elevator;
  }

  public void moveUp() {
    if (m_leftSparkFlex != null) {
      m_leftSparkFlex.set(Constants.ElevatorConstants.kElevatorSpeed);
    }
  }

  public void moveDown() {
    if (m_leftSparkFlex != null) {
      m_leftSparkFlex.set(-Constants.ElevatorConstants.kElevatorSpeed);
    }
  }

  public void stop() {
    if (m_leftSparkFlex != null) {
      m_leftSparkFlex.set(0.0);
    }
  }

  public double getAngle() {
    return m_absoluteEncoder.getPosition();
  }

  public void setTargetAngle(double angle) {
    angle = MathUtil.clamp(angle,
                              Constants.ElevatorConstants.kElevatorLowerLimitDegrees, 
                              Constants.ElevatorConstants.kElevatorUpperLimitDegrees);
    if (angle != m_lastAngle) {
      m_targetAngle.set(angle);
      m_lastAngle = angle;
      m_setpoint = new TrapezoidProfile.State(angle, 0.0);
    }
  }

  public void setTargetLevel(int level) {
    if (level < 1 || level > 4) return;
    setTargetAngle(Constants.ElevatorConstants.kElevatorLevels[level]);
  }

  @Override
  public void periodic() {
    if (Constants.ElevatorConstants.kEnableElevatorPIDTuning &&
        m_leftSparkFlex != null) {
      double tmp = m_TDelevatorP.get();
      boolean changed = false;
      if (tmp != m_elevatorP) {
        m_elevatorP = tmp;
        m_leftSparkFlexConfig.closedLoop.p(m_elevatorP);
        changed = true;
      }
      tmp = m_TDelevatorI.get();
      if (tmp != m_elevatorI) {
        m_elevatorI = tmp;
        changed = true;
        m_leftSparkFlexConfig.closedLoop.i(m_elevatorI);
      }
      tmp = m_TDelevatorD.get();
      if (tmp != m_elevatorD) {
        m_elevatorD = tmp;
        changed = true;
        m_leftSparkFlexConfig.closedLoop.d(m_elevatorD);
      }
      if(changed) {
        m_leftSparkFlex.configure(m_leftSparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }
    }
    if (RobotMap.E_ENABLED) {
      m_leftCurrentOutput.set(m_leftSparkFlex.getOutputCurrent());
      m_rightCurrentOutput.set(m_rightSparkFlex.getOutputCurrent());
      m_elevatorEncoderValueDegrees.set(getAngle()/Constants.ElevatorConstants.kElevatorEncoderPositionFactor);
      m_elevatorEncoderValueRotations.set(getAngle());

      m_state = m_profile.calculate(periodTime, m_state, m_setpoint);
      double arbFeedForward = m_elevatorFeedForwardController.calculate(m_setpoint.velocity);
      m_closedLoopController.setReference(m_setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFeedForward);
    }

    super.periodic();
  }
}
