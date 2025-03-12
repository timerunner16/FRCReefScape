// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDBoolean;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.testingdashboard.TDSendable;

public class Elevator extends SubsystemBase {
  private static Elevator m_elevator;

  private final double periodTime = 0.02;

  TDNumber m_targetAngle;
  TDNumber m_elevatorEncoderValueInches;
  TDNumber m_elevatorCurrentOutput;
  TDNumber m_TDelevatorP;
  TDNumber m_TDelevatorI;
  TDNumber m_TDelevatorD;
  TDNumber m_TDelevatorKs;
  TDNumber m_TDelevatorKg;
  TDNumber m_TDelevatorKv;
  TDNumber m_TDelevatorKa;
  TDNumber m_TDelevatorFFout;
  double m_elevatorP = Constants.ElevatorConstants.kElevatorP;
  double m_elevatorI = Constants.ElevatorConstants.kElevatorI;
  double m_elevatorD = Constants.ElevatorConstants.kElevatorD;
  double m_elevatorkS = Constants.ElevatorConstants.kElevatorkS;
  double m_elevatorkG = Constants.ElevatorConstants.kElevatorkG;
  double m_elevatorkV = Constants.ElevatorConstants.kElevatorkV;
  double m_elevatorkA = Constants.ElevatorConstants.kElevatorkA;
  private double m_elevatorLastAngle = 0;

  SparkFlex m_elevatorLeftSparkFlex;
  SparkFlex m_elevatorRightSparkFlex;

  SparkFlexConfig m_leftSparkFlexConfig;

  DigitalInput m_elevatorHighLimit;
  DigitalInput m_elevatorLowLimit;

  TDBoolean m_TDHighLimitHit;
  TDBoolean m_TDLowLimitHit;

  TDNumber m_elevatorLeftCurrentOutput;
  TDNumber m_elevatorRightCurrentOutput;

  SparkClosedLoopController m_elevatorClosedLoopController;
  RelativeEncoder m_elevatorMotorEncoder;

  ElevatorFeedforward m_elevatorFeedForwardController;
  TrapezoidProfile m_elevatorProfile;
  TrapezoidProfile.State m_elevatorState;
  TrapezoidProfile.State m_elevatorSetpoint;
  TDNumber m_TDelevatorProfiledVelocity;

  TDNumber m_targetShoulderAngle;
  TDNumber m_shoulderEncoderValueRotations;
  TDNumber m_shoulderEncoderValueDegrees;
  TDNumber m_shoulderCurrentOutput;
  TDNumber m_TDShoulderFFoutput;
  TDNumber m_TDshoulderKg;
  TDNumber m_TDshoulderKv;
  TDNumber m_TDshoulderKs;
  TDNumber m_TDshoulderP;
  TDNumber m_TDshoulderI;
  TDNumber m_TDshoulderD;
  double m_shoulderP = Constants.ElevatorConstants.kShoulderP;
  double m_shoulderI = Constants.ElevatorConstants.kShoulderI;
  double m_shoulderD = Constants.ElevatorConstants.kShoulderD;
  double m_shoulderkS = Constants.ElevatorConstants.kShoulderkS;
  double m_shoulderkG = Constants.ElevatorConstants.kShoulderkG;
  double m_shoulderkV = Constants.ElevatorConstants.kShoulderkV;
  private double m_shoulderLastAngle = 0;

  SparkMax m_WoSSparkMax;
  SparkMaxConfig m_SparkMaxConfig;

  SparkMax m_shoulderSparkMax;
  SparkMaxConfig m_shoulderSparkMaxConfig;
  RelativeEncoder m_shoulderEncoder;
  SparkClosedLoopController m_shoulderClosedLoopController;

  ArmFeedforward m_shoulderArmFeedForwardController;

  // This gearbox represents a gearbox containing 2 Vex 775pro motors.
  private final DCMotor m_elevatorMotor = DCMotor.getNEO(2);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(m_elevatorMotor,
       20,
       5.0,
       0.10,
       0.0,
       1.0,
       true,
       0.0);
  private Encoder m_encoder;
  private EncoderSim m_encoderSim;
  private DCMotorSim m_motorSim;

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters() * 20, 90));
  
  /** Creates a new Elevator. */
  private Elevator() {
    super("Elevator");

    if (RobotMap.E_ENABLED) {
      // setup elevator
      m_elevatorLeftSparkFlex = new SparkFlex(RobotMap.E_LEFTMOTOR, MotorType.kBrushless);
      m_elevatorRightSparkFlex = new SparkFlex(RobotMap.E_RIGHTMOTOR, MotorType.kBrushless);

      m_leftSparkFlexConfig = new SparkFlexConfig();
      SparkFlexConfig rightElevatorSparkFlexConfig = new SparkFlexConfig();

      rightElevatorSparkFlexConfig.follow(m_elevatorLeftSparkFlex, true);

      m_TDelevatorP = new TDNumber(this, "Elevator PID", "elevP", Constants.ElevatorConstants.kElevatorP);
      m_TDelevatorI = new TDNumber(this, "Elevator PID", "elevI", Constants.ElevatorConstants.kElevatorI);
      m_TDelevatorD = new TDNumber(this, "Elevator PID", "elevD", Constants.ElevatorConstants.kElevatorD);
      m_TDelevatorKg = new TDNumber(this, "Elevator PID", "elevkG", Constants.ElevatorConstants.kElevatorkG);
      m_TDelevatorKs = new TDNumber(this, "Elevator PID", "elevkS", Constants.ElevatorConstants.kElevatorkS);
      m_TDelevatorKv = new TDNumber(this, "Elevator PID", "elevkV", Constants.ElevatorConstants.kElevatorkV);
      m_TDelevatorKa = new TDNumber(this, "Elevator PID", "elevkA", Constants.ElevatorConstants.kElevatorkA);
      m_TDelevatorFFout = new TDNumber(this, "Elevator PID", "FF Out");

      m_leftSparkFlexConfig.closedLoop.pid(Constants.ElevatorConstants.kElevatorP, Constants.ElevatorConstants.kElevatorI,
          Constants.ElevatorConstants.kElevatorD);
      m_leftSparkFlexConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      m_leftSparkFlexConfig.closedLoop.positionWrappingEnabled(false);

      m_leftSparkFlexConfig.softLimit.forwardSoftLimit(Constants.ElevatorConstants.kElevatorUpperLimitInches);
      m_leftSparkFlexConfig.softLimit.reverseSoftLimit(Constants.ElevatorConstants.kElevatorLowerLimitInches);
      m_leftSparkFlexConfig.softLimit.forwardSoftLimitEnabled(true);
      m_leftSparkFlexConfig.softLimit.reverseSoftLimitEnabled(true);
      m_leftSparkFlexConfig.encoder.positionConversionFactor(Constants.ElevatorConstants.kElevatorEncoderPositionFactor);

      m_elevatorRightSparkFlex.configure(rightElevatorSparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_elevatorLeftSparkFlex.configure(m_leftSparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_elevatorLowLimit = new DigitalInput(RobotMap.E_LIMITLOW);
      m_elevatorHighLimit = new DigitalInput(RobotMap.E_LIMITHIGH);
      m_TDHighLimitHit = new TDBoolean(this , "Limits", "High Limit");
      m_TDLowLimitHit = new TDBoolean(this, "Limits", "Low Limit");

      m_elevatorClosedLoopController = m_elevatorLeftSparkFlex.getClosedLoopController();
      m_elevatorMotorEncoder = m_elevatorLeftSparkFlex.getEncoder();
      m_elevatorMotorEncoder.setPosition(0);

      m_elevatorFeedForwardController = new ElevatorFeedforward(Constants.ElevatorConstants.kElevatorkS, Constants.ElevatorConstants.kElevatorkG, Constants.ElevatorConstants.kElevatorkV, Constants.ElevatorConstants.kElevatorkA);
      m_elevatorProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        Constants.ElevatorConstants.kElevatorMaxVelocity,
        Constants.ElevatorConstants.kElevatorMaxAcceleration
      ));
      m_elevatorSetpoint = new TrapezoidProfile.State(m_elevatorMotorEncoder.getPosition(), 0.0);
      m_elevatorState = new TrapezoidProfile.State(m_elevatorMotorEncoder.getPosition(), 0.0);
      m_TDelevatorProfiledVelocity = new TDNumber(this, "Elevator PID", "Profile Velocity");

      m_targetAngle = new TDNumber(this, "Elevator Encoder Values", "Target Angle", getElevatorAngle());
      m_elevatorEncoderValueInches = new TDNumber(this, "Elevator Encoder Values", "Height (inches)", getElevatorAngle());
      m_elevatorLeftCurrentOutput = new TDNumber(this, "Current", "Left Elevator Output", m_elevatorLeftSparkFlex.getOutputCurrent());
      m_elevatorRightCurrentOutput = new TDNumber(this, "Current", "Right Elevator Output", m_elevatorRightSparkFlex.getOutputCurrent());

      
      // setup shoulder
      m_shoulderSparkMax = new SparkMax(RobotMap.E_SHOULDERMOTOR, MotorType.kBrushless);
      m_shoulderSparkMaxConfig = new SparkMaxConfig();

      m_TDshoulderP = new TDNumber(this, "WoS Shoulder PID", "P", Constants.ElevatorConstants.kShoulderP);
      m_TDshoulderI = new TDNumber(this, "WoS Shoulder PID", "I", Constants.ElevatorConstants.kShoulderI);
      m_TDshoulderD = new TDNumber(this, "WoS Shoulder PID", "D", Constants.ElevatorConstants.kShoulderD);
      m_TDshoulderKg = new TDNumber(this, "WoS Shoulder PID", "kG", Constants.ElevatorConstants.kShoulderkG);
      m_TDshoulderKv = new TDNumber(this, "WoS Shoulder PID", "kV", Constants.ElevatorConstants.kShoulderkV);
      m_TDshoulderKs = new TDNumber(this, "WoS Shoulder PID", "kS", Constants.ElevatorConstants.kShoulderkS);
      m_TDShoulderFFoutput = new TDNumber(this, "WoS Shoulder PID", "Shoulder FF Out");

      m_shoulderSparkMaxConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50, 80, 100);
      
      m_shoulderSparkMaxConfig.softLimit
                          .forwardSoftLimit(Constants.ElevatorConstants.kShoulderUpperLimitDegrees)
                          .forwardSoftLimitEnabled(true)
                          .reverseSoftLimit(Constants.ElevatorConstants.kShoulderLowerLimitDegrees)
                          .reverseSoftLimitEnabled(true);

      m_shoulderSparkMaxConfig.closedLoop.pid(Constants.ElevatorConstants.kShoulderP, Constants.ElevatorConstants.kShoulderI,
          Constants.ElevatorConstants.kShoulderD);
      m_shoulderSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

      m_shoulderSparkMaxConfig.encoder.positionConversionFactor(Constants.ElevatorConstants.kShoulderEncoderPositionFactor);

      m_shoulderSparkMax.configure(m_shoulderSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_shoulderEncoder = m_shoulderSparkMax.getEncoder();
      m_shoulderEncoder.setPosition(0);
      m_shoulderClosedLoopController = m_shoulderSparkMax.getClosedLoopController();

      m_shoulderArmFeedForwardController = new ArmFeedforward(m_shoulderkS, m_shoulderkG, m_shoulderkV);

      m_targetShoulderAngle = new TDNumber(this, "WoS Encoder Values", "Target Shoulder Angle", getShoulderAngle());
      m_shoulderEncoderValueRotations = new TDNumber(this, "WoS Encoder Values", "Rotations", getShoulderAngle() / Constants.ElevatorConstants.kShoulderEncoderPositionFactor);
      m_shoulderEncoderValueDegrees = new TDNumber(this, "WoS Encoder Values", "Shoulder Angle (radians)", getShoulderAngle());
      m_shoulderCurrentOutput = new TDNumber(this, "Current", "Shoulder Current", m_shoulderSparkMax.getOutputCurrent());

      m_encoder = new Encoder(8,9);
      m_encoderSim = new EncoderSim(m_encoder);
      m_motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(6, 8), m_elevatorMotor);

      new TDSendable(this, "Elevator", "Position", m_mech2d);
    }
  }

  public static Elevator getInstance() {
    if (m_elevator == null) {
      m_elevator = new Elevator();
    }
    return m_elevator;
  }

  /**
   * Move elevator under power control
   * @param commandedSpeed
   */
  public void moveElevator(double commandedSpeed)
  {

    double speed = commandedSpeed;
    if(m_elevatorLeftSparkFlex != null) {
      if(!m_elevatorHighLimit.get() && speed > 0)
      {
        speed = 0;
      }
      if(!m_elevatorLowLimit.get() && speed < 0)
      {
        speed = 0;
      }
      m_elevatorLeftSparkFlex.set(speed);
    }
  }

  public void moveElevatorUp() {
    if (m_elevatorLeftSparkFlex != null) {
      m_elevatorLeftSparkFlex.set(Constants.ElevatorConstants.kElevatorSpeed);
      m_motorSim.setInputVoltage(Constants.ElevatorConstants.kElevatorSpeed * 12);
    }
  }

  public void moveElevatorDown() {
    if (m_elevatorLeftSparkFlex != null) {
      m_elevatorLeftSparkFlex.set(-Constants.ElevatorConstants.kElevatorSpeed);
      m_motorSim.setInputVoltage(-Constants.ElevatorConstants.kElevatorSpeed * 12);
    }
  }

  public void stopElevator() {
    if (m_elevatorLeftSparkFlex != null) {
      m_elevatorLeftSparkFlex.set(0.0);
      m_motorSim.setInputVoltage(0);
    }
  }

  public double getElevatorTargetAngle() {
    return m_elevatorLastAngle;
  }

  public double getElevatorAngle() {
    return m_elevatorMotorEncoder.getPosition();
  }

  public boolean isElevatorAtGoal() {
    return MathUtil.isNear(m_elevatorLastAngle, getElevatorAngle(), Constants.ElevatorConstants.kElevatorToleranceInches);
  }

  public void setElevatorTargetAngle(double positionInches) {
    positionInches = MathUtil.clamp(positionInches,
                              Constants.ElevatorConstants.kElevatorLowerLimitInches, 
                              Constants.ElevatorConstants.kElevatorUpperLimitInches);
    if (positionInches != m_elevatorLastAngle) {
      m_targetAngle.set(positionInches);
      m_elevatorLastAngle = positionInches;
      m_elevatorSetpoint = new TrapezoidProfile.State(positionInches, 0.0);
    }
  }

  public void setElevatorTargetLevel(int level) {
    int maxIndex = (Constants.ElevatorConstants.kShoulderLevels.length - 1);
    if (level < 0 || level > maxIndex) return;
    setElevatorTargetAngle(Constants.ElevatorConstants.kElevatorLevels[level]);
  }

  public double getShoulderTargetAngle() {
    return m_shoulderLastAngle;
  }

  public double getShoulderAngle() { 
    return m_shoulderEncoder.getPosition();
  }

  public boolean isShoulderAtGoal() {
    return MathUtil.isNear(m_shoulderLastAngle, getShoulderAngle(), Constants.ElevatorConstants.kShoulderToleranceDegrees);
  }

  public void setShoulderTargetAngle(double angle) {
    double setpoint = angle % Constants.ElevatorConstants.DEGREES_PER_REVOLUTION;
    setpoint = MathUtil.clamp(setpoint,
                              Constants.ElevatorConstants.kShoulderLowerLimitDegrees, 
                              Constants.ElevatorConstants.kShoulderUpperLimitDegrees);
    if (setpoint != m_shoulderLastAngle) {
      m_targetShoulderAngle.set(setpoint);
      m_shoulderLastAngle = setpoint;
    }
  }

  public void setShoulderTargetLevel(int level) {
    int maxIndex = (Constants.ElevatorConstants.kShoulderLevels.length - 1);
    if (level < 0 || level > maxIndex) return;
    setShoulderTargetAngle(Constants.ElevatorConstants.kShoulderLevels[level]);
  }

  public void setShoulderSpeeds(double ShoulderRPM) {
      m_WoSSparkMax.getClosedLoopController().setReference(ShoulderRPM, ControlType.kVelocity);
  }

  public void spinShoulder(double shoulderSpeed){
    if (m_shoulderSparkMax != null) {
      m_shoulderSparkMax.set(shoulderSpeed);
    }
  }

  public void stopShoulder(){
    if (m_shoulderSparkMax != null) {
      m_shoulderSparkMax.set(0);
    }
  }

  public boolean inGoalPosition() {
    return isElevatorAtGoal() && isShoulderAtGoal();
  }

  @Override
  public void periodic() {
    if (Constants.ElevatorConstants.kEnableElevatorPIDTuning &&
        m_elevatorLeftSparkFlex != null) {
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
        m_elevatorLeftSparkFlex.configure(m_leftSparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }

      boolean ffchanged = false;
      tmp = m_TDelevatorKg.get();
      if(tmp != m_elevatorkG) {
        m_elevatorkG = tmp;
        ffchanged = true;
      }
      tmp = m_TDelevatorKv.get();
      if(tmp != m_elevatorkV) {
        m_elevatorkV = tmp;
        ffchanged = true;
      }
      tmp = m_TDelevatorKs.get();
      if(tmp != m_elevatorkS)
      {
        m_elevatorkS = tmp;
        ffchanged = true;
      }
      tmp = m_TDelevatorKa.get();
      if(tmp != m_elevatorkA)
      {
        m_elevatorkA = tmp;
        ffchanged = true;
      }
      if(ffchanged) {
        m_elevatorFeedForwardController = new ElevatorFeedforward(m_elevatorkS, m_elevatorkG, m_elevatorkV, m_elevatorkA);
      }
      tmp = m_targetAngle.get();
      if(tmp != m_elevatorLastAngle)
      {
        setElevatorTargetAngle(tmp);
      }
    }
    if (Constants.ElevatorConstants.kEnableShoulderPIDTuning &&
        m_shoulderSparkMax != null) {
      double tmp = m_TDshoulderP.get();
      boolean changed = false;
      if (tmp != m_shoulderP) {
        m_shoulderP = tmp;
        m_shoulderSparkMaxConfig.closedLoop.p(m_shoulderP);
        changed = true;
      }
      tmp = m_TDshoulderI.get();
      if (tmp != m_shoulderI) {
        m_shoulderI = tmp;
        changed = true;
        m_shoulderSparkMaxConfig.closedLoop.i(m_shoulderI);
      }
      tmp = m_TDshoulderD.get();
      if (tmp != m_shoulderD) {
        m_shoulderD = tmp;
        changed = true;
        m_shoulderSparkMaxConfig.closedLoop.d(m_shoulderD);
      }
      if(changed) {
        m_shoulderSparkMax.configure(m_shoulderSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }

      boolean ffchanged = false;
      tmp = m_TDshoulderKg.get();
      if(tmp != m_shoulderkG) {
        m_shoulderkG = tmp;
        ffchanged = true;
      }
      tmp = m_TDshoulderKv.get();
      if(tmp != m_shoulderkV) {
        m_shoulderkV = tmp;
        ffchanged = true;
      }
      tmp = m_TDshoulderKs.get();
      if(tmp != m_shoulderkS)
      {
        m_shoulderkS = tmp;
        ffchanged = true;
      }
      if(ffchanged) {
        m_shoulderArmFeedForwardController = new ArmFeedforward(m_shoulderkS, m_shoulderkG, m_shoulderkV);
      }
      tmp = m_targetShoulderAngle.get();
      if(tmp != m_shoulderLastAngle)
      {
        setShoulderTargetAngle(tmp);
      }
    }
    if (RobotMap.E_ENABLED) {
      m_elevatorLeftCurrentOutput.set(m_elevatorLeftSparkFlex.getOutputCurrent());
      m_elevatorRightCurrentOutput.set(m_elevatorRightSparkFlex.getOutputCurrent());
      m_elevatorEncoderValueInches.set(getElevatorAngle());

      m_shoulderCurrentOutput.set(m_shoulderSparkMax.getOutputCurrent());
      m_shoulderEncoderValueDegrees.set(getShoulderAngle());
      m_shoulderEncoderValueRotations.set(getShoulderAngle());

      if(Constants.ElevatorConstants.kEnableShoulderClosedLoopControl){
        double shoulderArbFeedforward = m_shoulderArmFeedForwardController.calculate(Units.degreesToRadians(m_shoulderLastAngle + 90), 0.0);
        m_TDShoulderFFoutput.set(shoulderArbFeedforward);
        m_shoulderClosedLoopController.setReference(m_shoulderLastAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0, shoulderArbFeedforward);
      }
      if(Constants.ElevatorConstants.kEnableElevatorClosedLoopControl){
        m_elevatorState = m_elevatorProfile.calculate(periodTime, m_elevatorState, m_elevatorSetpoint);
        m_TDelevatorProfiledVelocity.set(m_elevatorState.position);
        double elevatorArbFeedforward = m_elevatorFeedForwardController.calculate(m_elevatorState.velocity);
        m_TDelevatorFFout.set(elevatorArbFeedforward);
        m_elevatorClosedLoopController.setReference(m_elevatorSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, elevatorArbFeedforward);
      }

      m_elevatorMech2d.setLength(m_elevatorSim.getPositionMeters() * 20);
      m_TDHighLimitHit.set(m_elevatorHighLimit.get());
      m_TDLowLimitHit.set(m_elevatorLowLimit.get());
    }

    super.periodic();
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getInputVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters() * 10);
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  public void setShoulderVoltage(Voltage volts) {
    m_shoulderSparkMax.setVoltage(volts.in(BaseUnits.VoltageUnit));
  }

  public double getShoulderVoltage() {
    return m_shoulderSparkMax.get() * RobotController.getBatteryVoltage();
  }

  public double getShoulderPosition() {
    return m_shoulderSparkMax.getEncoder().getPosition();
  }

  public double getShoulderVelocity() {
    return m_shoulderSparkMax.getEncoder().getVelocity();
  }
}
