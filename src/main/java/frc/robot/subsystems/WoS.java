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

  SparkMax m_WoSSparkMax;
  SparkMaxConfig m_SparkMaxConfig;

  /** Creates a new ExampleSubsystem. */
  private WoS() {
    super("WoS");

    if (RobotMap.W_ENABLED) {
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
    if (RobotMap.W_ENABLED) {
      m_WoSCurrentOutput.set(m_WoSSparkMax.getEncoder().getVelocity());
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
}
