/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.GameDataHolder;
import frc.robot.subsystems.WoS;
import frc.robot.commands.drive.AlignToClosestReefLeft;
import frc.robot.commands.drive.AlignToClosestReefRight;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.SlowSwerveDrive;
import frc.robot.commands.drive.TargetDrive;
import frc.robot.commands.elevator.ElevatorManualPowerControl;
import frc.robot.commands.elevator.Jesus;
import frc.robot.commands.elevator.Lucifer;
import frc.robot.commands.elevator.SetElevatorAlgaeRemoveHigh;
import frc.robot.commands.elevator.SetElevatorAlgaeRemoveLow;
import frc.robot.commands.elevator.SetElevatorLevel;
import frc.robot.commands.elevator.ShoulderPowerControl;
import frc.robot.commands.funnel.Explode;
import frc.robot.commands.funnel.Implode;
import frc.robot.commands.FeedingTime;
import frc.robot.commands.Hurl;
import frc.robot.commands.AlgaeIntake.Nibble;
import frc.robot.commands.AlgaeIntake.Spit;
import frc.robot.commands.Climb.ClimberIn;
import frc.robot.commands.Climb.ClimberOut;
import frc.robot.commands.WoS.*;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.SwerveDriveInputs;
import frc.robot.utils.TargetPose;
import frc.robot.utils.FieldUtils.ReefFaceOffset;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private static OI m_oi;

  private static XboxController m_DriverXboxController;
  private static XboxController m_OperatorXboxController;
  private static GenericHID m_ReefController;

  private SwerveDriveInputs m_driveInputs;

  /**
   * Used outside of the OI class to return an instance of the class.
   * @return Returns instance of OI class formed from constructor.
   */
  public static OI getInstance() {
    if (m_oi == null) {
      m_oi = new OI();
    }
    return m_oi;
  }

  public OI() {
    // User Input
    // TODO: Tune deadband
    m_DriverXboxController = new XboxController(RobotMap.U_DRIVER_XBOX_CONTROLLER);
    m_OperatorXboxController = new XboxController(RobotMap.U_OPERATOR_XBOX_CONTROLLER);
    m_ReefController = new GenericHID(RobotMap.U_REEF_CONTROLLER);

    // Set up drive translation and rotation inputs
    XboxController driveController = m_DriverXboxController;
    Supplier<Double> xInput;
    Supplier<Double> yInput;
    if(RobotBase.isReal()){
      xInput = ()->driveController.getLeftY();
      yInput = ()->driveController.getLeftX();
    } else {
      xInput = ()->-driveController.getLeftX();
      yInput = ()->driveController.getLeftY();
    }
    m_driveInputs = new SwerveDriveInputs(xInput, yInput, ()->driveController.getRightX());
  }

  public void bindControls() {
    ////////////////////////////////////////////////////
    // Now Mapping Commands to XBox
    ////////////////////////////////////////////////////

    // Driver Grease Man's Special Abilities(OP)
    new JoystickButton(m_DriverXboxController, Button.kBack.value).onTrue(new InstantCommand(()->Drive.getInstance().zeroHeading()));
    new JoystickButton(m_DriverXboxController, Button.kA.value).whileTrue(
      new TargetDrive(()->{return FieldUtils.getInstance().getTagPose(
        FieldUtils.getInstance().getAllianceAprilTags().middleFrontReef).toPose2d();}, m_driveInputs));
        
    new JoystickButton(m_DriverXboxController, Button.kY.value).whileTrue(new DriveToPose(FieldUtils.getInstance()::getRedCoralA1Pose)); 
    new JoystickButton(m_DriverXboxController, Button.kX.value).whileTrue(new DriveToPose(FieldUtils.getInstance()::getRedCoralA2Pose));
    new JoystickButton(m_DriverXboxController, Button.kLeftBumper.value).whileTrue(new AlignToClosestReefLeft());
    new JoystickButton(m_DriverXboxController, Button.kRightBumper.value).whileTrue(new AlignToClosestReefRight());
      new Trigger(()->{return (m_DriverXboxController.getLeftTriggerAxis() > 0.5);}).whileTrue(new SlowSwerveDrive(m_driveInputs));
    
    //Operator Cookie Monster Special Abilities(MEGA OP)
    new JoystickButton(m_OperatorXboxController, Button.kX.value).onTrue(new SetElevatorLevel(4));
    new JoystickButton(m_OperatorXboxController, Button.kY.value).onTrue(new SetElevatorLevel(3));
    new JoystickButton(m_OperatorXboxController, Button.kB.value).onTrue(new SetElevatorLevel(2));
    new JoystickButton(m_OperatorXboxController, Button.kA.value).onTrue(new SetElevatorLevel(1));
    new JoystickButton(m_OperatorXboxController, Button.kLeftBumper.value).whileTrue(new Consume());

    new Trigger(()->{return (m_OperatorXboxController.getLeftTriggerAxis() > 0.5);}).whileTrue(new FeedingTime());
    new Trigger(()->{return (m_OperatorXboxController.getRightTriggerAxis() > 0.5);}).whileTrue(new Hurl());
    new Trigger(m_OperatorXboxController.povUp(CommandScheduler.getInstance().getDefaultButtonLoop())).whileTrue(new SetElevatorAlgaeRemoveHigh());
    new Trigger(m_OperatorXboxController.povDown(CommandScheduler.getInstance().getDefaultButtonLoop())).whileTrue(new SetElevatorAlgaeRemoveLow());
    new Trigger(m_OperatorXboxController.povLeft(CommandScheduler.getInstance().getDefaultButtonLoop())).whileTrue(new ClimberIn());
    new Trigger(m_OperatorXboxController.povRight(CommandScheduler.getInstance().getDefaultButtonLoop())).whileTrue(new ClimberOut());

    //Reef Controller (AAAUUGGHHGH)
    new JoystickButton(m_ReefController, 1).onTrue(
      GameDataHolder.getInstance().SetDisplayLevel1());
    new JoystickButton(m_ReefController, 2).onTrue(
      GameDataHolder.getInstance().SetDisplayLevel2());
    new JoystickButton(m_ReefController, 3).onTrue(
      GameDataHolder.getInstance().SetDisplayLevel3());
    new JoystickButton(m_ReefController, 4).onTrue(
      GameDataHolder.getInstance().SetDisplayLevel4());
    for (int i = 0; i < 12; i++) {
      new JoystickButton(m_ReefController, i+5).onTrue(
        GameDataHolder.getInstance().SetStateCommand(i));
    }
  };
  

  /**
   * Returns the Xbox Controller
   * @return the Xbox Controller
   */
  public XboxController getDriverXboxController() {
      return m_DriverXboxController;
  }

  public XboxController getOperatorXboxController() {
    return m_OperatorXboxController;
  }

  /**
   * Returns the reef state controller
   * @return the Controller
   */
  public GenericHID getReefController() {
    return m_ReefController;
  }

  public SwerveDriveInputs getDriveInputs() {
    return m_driveInputs;
  }
}

