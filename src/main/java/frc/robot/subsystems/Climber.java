// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.TDNumber;

public class Climber extends SubsystemBase {
  private static Climber m_Climber;
  TDNumber m_TDwinchP;
  TDNumber m_TDwinchI;
  TDNumber m_TDwinchD;
  double m_winchP = Constants.ClimberConstants.kWinchP;
  double m_winchI = Constants.ClimberConstants.kWinchI;
  double m_winchD = Constants.ClimberConstants.kWinchD;
  
  SparkMax m_WinchMotor;

  SparkMaxConfig m_WinchMotorConfig;

  TDNumber m_ClimberCurrentOutput;

  /** Creates a new Climber. */
  public Climber() {
    super("Climber");

    if (RobotMap.C_ENABLED) {
      m_WinchMotor = new SparkMax(RobotMap.C_WINCHMOTOR, MotorType.kBrushless);

      m_WinchMotorConfig = new SparkMaxConfig();
      m_WinchMotorConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(40, 60);

      m_TDwinchP = new TDNumber(this, "PID", "P", Constants.ClimberConstants.kWinchP);
      m_TDwinchI = new TDNumber(this, "PID", "I", Constants.ClimberConstants.kWinchI);
      m_TDwinchD = new TDNumber(this, "PID", "D", Constants.ClimberConstants.kWinchD);

      m_WinchMotorConfig.closedLoop.pid(Constants.ClimberConstants.kWinchP, Constants.ClimberConstants.kWinchI, 
        Constants.ClimberConstants.kWinchD);
      m_WinchMotor.configure(m_WinchMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
      m_ClimberCurrentOutput = new TDNumber(this, "Climber", "Motor Current");
    }
  }

  public static Climber getInstance() {
    if (m_Climber == null) {
      m_Climber = new Climber();
    }
    return m_Climber;
  }

  public void setSpeed(double RPM, boolean backwards) {
    if (!backwards) {
      m_WinchMotor.getClosedLoopController().setReference(RPM, ControlType.kVelocity);
    } else {
      m_WinchMotor.getClosedLoopController().setReference(-RPM, ControlType.kVelocity);
    }
  }

  public void winchIn(double speed) {
    if (m_WinchMotor != null) {
      m_WinchMotor.set(speed);
    }
  }

  public void winchOut(double speed) {
    if (m_WinchMotor != null) {
      m_WinchMotor.set(-speed);
    }
  }

  public void stopWinch() {
    if (m_WinchMotor != null) {
      m_WinchMotor.set(0);
    }
  }

  @Override
  public void periodic() {
    if (Constants.ClimberConstants.kEnableWinchPIDTuning && m_WinchMotor != null) {
      double tmp = m_TDwinchP.get();
      boolean changed = false;
      if (tmp != m_winchP) {
        m_winchP = tmp;
        m_WinchMotorConfig.closedLoop.p(m_winchP);
        changed = true;
      }
      tmp = m_TDwinchI.get();
      if (tmp != m_winchI) {
        m_winchI = tmp;
        m_WinchMotorConfig.closedLoop.i(m_winchI);
        changed = true;
      }
      tmp = m_TDwinchD.get();
      if (tmp != m_winchD) {
        m_winchD = tmp;
        m_WinchMotorConfig.closedLoop.d(m_winchD);
        changed = true;
      }
      if (changed) {
        m_WinchMotor.configure(m_WinchMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }
    }
    if (RobotMap.C_ENABLED) {
      m_ClimberCurrentOutput.set(m_WinchMotor.getOutputCurrent());
    }

    super.periodic();
  }
}
