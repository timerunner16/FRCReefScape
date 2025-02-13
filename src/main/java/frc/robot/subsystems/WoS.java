// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants;
import frc.robot.RobotMap;


public class WoS extends SubsystemBase {
  private static WoS m_WoS;

  TDNumber m_TDwheelP;
  TDNumber m_TDwheelI;
  TDNumber m_TDwheelD;
  TDNumber m_WoSCurrentOutput;
  double m_wheelP = Constants.WoSConstants.kWoSP;
  double m_wheelI = Constants.WoSConstants.kWoSI;
  double m_wheelD = Constants.WoSConstants.kWoSD;

  TDNumber m_targetAngle;
  TDNumber m_WoSShoulderEncoderValueRotations;
  TDNumber m_WoSShoulderEncoderValueDegrees;
  TDNumber m_WoSShoulderCurrentOutput;
  TDNumber m_TDshoulderP;
  TDNumber m_TDshoulderI;
  TDNumber m_TDshoulderD;
  double m_shoulderP = Constants.WoSConstants.kShoulderP;
  double m_shoulderI = Constants.WoSConstants.kShoulderI;
  double m_shoulderD = Constants.WoSConstants.kShoulderD;
  double m_shoulderkS = Constants.WoSConstants.kShoulderkS;
  double m_shoulderkG = Constants.WoSConstants.kShoulderkG;
  double m_shoulderkV = Constants.WoSConstants.kShoulderkV;
  private double m_lastAngle = 0;

  SparkMax m_WoSSparkMax;
  SparkMaxConfig m_SparkMaxConfig;

  SparkMax m_WoSShoulderSparkMax;
  SparkMaxConfig m_WoSShoulderSparkMaxConfig;
  SparkAbsoluteEncoder m_WoSShoulderAbsoluteEncoder;
  SparkClosedLoopController m_WoSShoulderClosedLoopController;

  ArmFeedforward m_WoSShoulderArmFeedForwardController;

  /** Creates a new ExampleSubsystem. */
  private WoS() {
    super("WoS");

    if (RobotMap.W_ENABLED) {
      // setup wheels
      m_WoSSparkMax = new SparkMax(RobotMap.W_MOTOR, MotorType.kBrushless);
      m_SparkMaxConfig = new SparkMaxConfig();

      SparkMaxConfig WoSSparkMaxConfig = new SparkMaxConfig();

      m_TDwheelP = new TDNumber(this, "Wheel PID", "P", Constants.WoSConstants.kWoSP);
      m_TDwheelI = new TDNumber(this, "Wheel PID", "I", Constants.WoSConstants.kWoSI);
      m_TDwheelD = new TDNumber(this, "Wheel PID", "D", Constants.WoSConstants.kWoSD);

      WoSSparkMaxConfig.closedLoop.pid(Constants.WoSConstants.kWoSP, Constants.WoSConstants.kWoSI,
          Constants.WoSConstants.kWoSD);
      m_WoSSparkMax.configure(WoSSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_WoSCurrentOutput = new TDNumber(this, "WoS", "Motor Current");

      
      // setup shoulder
      m_WoSShoulderSparkMax = new SparkMax(RobotMap.W_SHOULDERMOTOR, MotorType.kBrushless);
      m_WoSShoulderSparkMaxConfig = new SparkMaxConfig();

      m_TDshoulderP = new TDNumber(this, "WoS Shoulder PID", "P", Constants.WoSConstants.kShoulderP);
      m_TDshoulderI = new TDNumber(this, "WoS Shoulder PID", "I", Constants.WoSConstants.kShoulderI);
      m_TDshoulderD = new TDNumber(this, "WoS Shoulder PID", "D", Constants.WoSConstants.kShoulderD);

      m_WoSShoulderSparkMaxConfig.idleMode(IdleMode.kBrake);

      m_WoSShoulderSparkMaxConfig.closedLoop.pid(Constants.WoSConstants.kShoulderP, Constants.WoSConstants.kShoulderI,
          Constants.WoSConstants.kShoulderD);
      m_WoSShoulderSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      m_WoSShoulderSparkMaxConfig.closedLoop.positionWrappingEnabled(true);
      m_WoSShoulderSparkMaxConfig.closedLoop.positionWrappingMinInput(0);
      m_WoSShoulderSparkMaxConfig.closedLoop.positionWrappingMaxInput(Constants.WoSConstants.DEGREES_PER_REVOLUTION);

      m_WoSShoulderSparkMaxConfig.absoluteEncoder.positionConversionFactor(Constants.WoSConstants.kWoSShoulderEncoderPositionFactor);
      m_WoSShoulderSparkMaxConfig.absoluteEncoder.velocityConversionFactor(Constants.WoSConstants.kWoSShoulderEncoderVelocityFactor);
      m_WoSShoulderSparkMaxConfig.absoluteEncoder.inverted(false);

      m_WoSShoulderSparkMax.configure(m_WoSShoulderSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_WoSShoulderAbsoluteEncoder = m_WoSShoulderSparkMax.getAbsoluteEncoder();

      m_WoSShoulderArmFeedForwardController = new ArmFeedforward(m_shoulderkS, m_shoulderkG, m_shoulderkV);

      m_targetAngle = new TDNumber(this, "WoS Encoder Values", "Target Angle", getAngle());
      m_WoSShoulderEncoderValueRotations = new TDNumber(this, "WoS Encoder Values", "Rotations", getAngle() / Constants.WoSConstants.kWoSShoulderEncoderPositionFactor);
      m_WoSShoulderEncoderValueDegrees = new TDNumber(this, "WoS Encoder Values", "Angle (degrees)", getAngle());
      m_WoSShoulderCurrentOutput = new TDNumber(Drive.getInstance(), "Current", "WoS Angle Output", m_WoSShoulderSparkMax.getOutputCurrent());
    }
  }

  public static WoS getInstance() {
    if (m_WoS == null) {
      m_WoS = new WoS();
    }
    return m_WoS;
  }

  public void setSpeeds(double RPM, boolean backwards) {
    if (!backwards) {
      m_WoSSparkMax.getClosedLoopController().setReference(RPM, ControlType.kVelocity);
    } else {
      m_WoSSparkMax.getClosedLoopController().setReference(-RPM, ControlType.kVelocity);
    }
  }

  public void spinIn(double speed) {
    if (m_WoSSparkMax != null) {
      m_WoSSparkMax.set(speed);
    }
  }

  public void spinOut(double speed) {
    if (m_WoSSparkMax != null) {
      m_WoSSparkMax.set(-speed);
    }
  }

  public void noSpin(double speed) {
    if (m_WoSSparkMax != null) {
      m_WoSSparkMax.set(0);
    }
  }

  public double getAngle() {
    return m_WoSShoulderAbsoluteEncoder.getPosition();
  }

  public void setTargetAngle(double angle) {
    double setpoint = angle % Constants.WoSConstants.DEGREES_PER_REVOLUTION;
    setpoint = MathUtil.clamp(setpoint,
                              Constants.WoSConstants.kAngleLowerLimitDegrees, 
                              Constants.WoSConstants.kAngleUpperLimitDegrees);
    if (setpoint != m_lastAngle) {
      m_targetAngle.set(setpoint);
      m_lastAngle = setpoint;
    }
  }

  public void setTargetLevel(int level) {
    if (level < 1 || level > 4) return;
    setTargetAngle(Constants.WoSConstants.kWoSShoulderLevels[level]);
  }


  @Override
  public void periodic() {
    if (Constants.WoSConstants.kEnableWheelPIDTuning &&
        m_WoSSparkMax != null) {
      double tmp = m_TDwheelP.get();
      boolean changed = false;
      if (tmp != m_wheelP) {
        m_wheelP = tmp;
        m_SparkMaxConfig.closedLoop.p(m_wheelP);
        changed = true;
      }
      tmp = m_TDwheelI.get();
      if (tmp != m_wheelI) {
        m_wheelI = tmp;
        changed = true;
        m_SparkMaxConfig.closedLoop.i(m_wheelI);
      }
      tmp = m_TDwheelD.get();
      if (tmp != m_wheelD) {
        m_wheelD = tmp;
        changed = true;
        m_SparkMaxConfig.closedLoop.d(m_wheelD);
      }
      if(changed) {
        m_WoSSparkMax.configure(m_SparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }
    }
    if (Constants.WoSConstants.kEnableShoulderPIDTuning &&
        m_WoSShoulderSparkMax != null) {
      double tmp = m_TDshoulderP.get();
      boolean changed = false;
      if (tmp != m_shoulderP) {
        m_shoulderP = tmp;
        m_WoSShoulderSparkMaxConfig.closedLoop.p(m_shoulderP);
        changed = true;
      }
      tmp = m_TDshoulderI.get();
      if (tmp != m_shoulderI) {
        m_shoulderI = tmp;
        changed = true;
        m_WoSShoulderSparkMaxConfig.closedLoop.i(m_shoulderI);
      }
      tmp = m_TDshoulderD.get();
      if (tmp != m_shoulderD) {
        m_shoulderD = tmp;
        changed = true;
        m_WoSShoulderSparkMaxConfig.closedLoop.d(m_shoulderD);
      }
      if(changed) {
        m_WoSShoulderSparkMax.configure(m_WoSShoulderSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }
    }
    if (RobotMap.W_ENABLED) {
      m_WoSCurrentOutput.set(m_WoSSparkMax.getEncoder().getVelocity());
      m_WoSShoulderCurrentOutput.set(m_WoSShoulderSparkMax.getOutputCurrent());
      m_WoSShoulderEncoderValueDegrees.set(getAngle()/Constants.WoSConstants.kWoSShoulderEncoderPositionFactor);
      m_WoSShoulderEncoderValueRotations.set(getAngle());

      double arbFeedforward = m_WoSShoulderArmFeedForwardController.calculate(m_lastAngle, 0.0);
      m_WoSShoulderClosedLoopController.setReference(m_lastAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFeedforward);
    }

    super.periodic();
  }

  public void setRollerVoltage(Voltage volts) {
    m_WoSSparkMax.setVoltage(volts.in(BaseUnits.VoltageUnit));
  }

  public double getRollerVoltage() {
    return m_WoSSparkMax.get() * RobotController.getBatteryVoltage();
  }

  public double getRollerPosition() {
    return m_WoSSparkMax.getEncoder().getPosition();
  }

  public double getRollerVelocity() {
    return m_WoSSparkMax.getEncoder().getVelocity();
  }

  public void setShoulderVoltage(Voltage volts) {
    m_WoSShoulderSparkMax.setVoltage(volts.in(BaseUnits.VoltageUnit));
  }

  public double getShoulderVoltage() {
    return m_WoSShoulderSparkMax.get() * RobotController.getBatteryVoltage();
  }

  public double getShoulderPosition() {
    return m_WoSShoulderSparkMax.getEncoder().getPosition();
  }

  public double getShoulderVelocity() {
    return m_WoSShoulderSparkMax.getEncoder().getVelocity();
  }
}
