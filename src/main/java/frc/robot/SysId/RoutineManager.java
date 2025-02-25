// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SysId;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.OI;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDSendable;

/** Add your docs here. */
public class RoutineManager extends SubsystemBase {
    private static RoutineManager m_instance;
    private Map<String, Command> m_quasistaticForwardMap;
    private Map<String, Command> m_quasistaticReverseMap;
    private Map<String, Command> m_dynamicForwardMap;
    private Map<String, Command> m_dynamicReverseMap;
    private SendableChooser<String> m_routineChooser;

    private RoutineManager() {
        super("RoutineManager");
        m_quasistaticForwardMap = new HashMap<String,Command>();
        m_quasistaticReverseMap = new HashMap<String,Command>();
        m_dynamicForwardMap = new HashMap<String,Command>();
        m_dynamicReverseMap = new HashMap<String,Command>();
    }

    public static RoutineManager getInstance() {
        if(m_instance == null) {
            m_instance = new RoutineManager();
        }
        return m_instance;
    }

    public void mapRoutinesToController() {
        XboxController controller = OI.getInstance().getOperatorXboxController();

        new JoystickButton(controller, Button.kA.value).whileTrue(new SelectCommand<String>(m_quasistaticForwardMap,  ()->m_routineChooser.getSelected()));
        new JoystickButton(controller, Button.kB.value).whileTrue(new SelectCommand<String>(m_quasistaticReverseMap, ()->m_routineChooser.getSelected()));
        new JoystickButton(controller, Button.kX.value).whileTrue(new SelectCommand<String>(m_dynamicForwardMap, ()->m_routineChooser.getSelected()));
        new JoystickButton(controller, Button.kY.value).whileTrue(new SelectCommand<String>(m_dynamicReverseMap, ()->m_routineChooser.getSelected()));
    }

    public void registerRoutines() {
        m_routineChooser = new SendableChooser<String>();

        WosRollerRoutine wosRoller = new WosRollerRoutine();
        m_dynamicForwardMap.put("WoSRoller", wosRoller.dynamicForward());
        m_dynamicReverseMap.put("WoSRoller", wosRoller.dynamicReverse());
        m_quasistaticForwardMap.put("WoSRoller", wosRoller.quasistaticForward());
        m_quasistaticReverseMap.put("WoSRoller", wosRoller.quasistaticReverse());
        m_routineChooser.addOption("WoS Roller", "WoSRoller");

        WosShoulderRoutine wosShoulder = new WosShoulderRoutine();
        m_dynamicForwardMap.put("WoSShoulder", wosShoulder.dynamicForward());
        m_dynamicReverseMap.put("WoSShoulder", wosShoulder.dynamicReverse());
        m_quasistaticForwardMap.put("WoSShoulder", wosShoulder.quasistaticForward());
        m_quasistaticReverseMap.put("WoSShoulder", wosShoulder.quasistaticReverse());
        m_routineChooser.addOption("WoS Shoulder", "WoSRoller");

        new TDSendable(this, "Routines", "Routine Chooser", m_routineChooser);   
        SmartDashboard.putData(m_routineChooser);
     }
}
