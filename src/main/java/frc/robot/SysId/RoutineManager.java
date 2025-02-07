// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SysId;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.OI;

/** Add your docs here. */
public class RoutineManager {
    private static RoutineManager m_instance;
    private SendableChooser<SubsystemRoutine> m_routineChooser;

    private RoutineManager() {
        registerRoutines();
    }

    public static RoutineManager getInstance() {
        if(m_instance == null) {
            m_instance = new RoutineManager();
        }
        return m_instance;
    }

    public void mapRoutinesToController() {
        XboxController controller = OI.getInstance().getOperatorXboxController();
        
        new JoystickButton(controller, Button.kA.value).whileTrue(new ProxyCommand(()->{
            return m_routineChooser.getSelected().quasistaticForward();
        }));
        new JoystickButton(controller, Button.kB.value).whileTrue(new ProxyCommand(()->{
            return m_routineChooser.getSelected().quasistaticReverse();
        }));
        new JoystickButton(controller, Button.kX.value).whileTrue(new ProxyCommand(()->{
            return m_routineChooser.getSelected().dynamicForward();
        }));
        new JoystickButton(controller, Button.kY.value).whileTrue(new ProxyCommand(()->{
            return m_routineChooser.getSelected().dynamicReverse();
        }));
    }

    private void registerRoutines() {
        m_routineChooser = new SendableChooser<SubsystemRoutine>();

        m_routineChooser.addOption("Shooter", new WosRollerRoutine());
    }
}
