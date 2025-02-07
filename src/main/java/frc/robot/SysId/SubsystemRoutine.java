// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SysId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/** Add your docs here. */
public abstract class SubsystemRoutine {
    protected SysIdRoutine m_routine;
    private Command noRoutineCommand = new InstantCommand(()->{System.out.println("SysIdRoutine Not Defined");});

    public SubsystemRoutine(){}

    public Command quasistaticForward() {
        if(m_routine != null) {
            return m_routine.quasistatic(Direction.kForward);
        } else {
            return noRoutineCommand;
        }
    }

    public Command quasistaticReverse() {
        if(m_routine != null) {
            return m_routine.quasistatic(Direction.kReverse);
        } else {
            return noRoutineCommand;
        }
    }

    public Command dynamicForward() {
        if(m_routine != null) {
            return m_routine.dynamic(Direction.kForward);
        } else {
            return noRoutineCommand;
        }
    }

    public Command dynamicReverse() {
        if(m_routine != null) {
        return m_routine.dynamic(Direction.kReverse);
        } else {
            return noRoutineCommand;
        }
    }
}
