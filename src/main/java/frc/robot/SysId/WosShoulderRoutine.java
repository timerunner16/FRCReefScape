package frc.robot.SysId;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Elevator;

public class WosShoulderRoutine extends SubsystemRoutine {
    private Elevator m_elevator;

    private MutVoltage m_appliedVoltage = Volts.mutable(0);
    private MutAngle m_shoulderPosition = Radians.mutable(0);
    private MutAngularVelocity m_shoulderVelocity = RadiansPerSecond.mutable(0);

    public WosShoulderRoutine() {
        m_elevator = Elevator.getInstance();

        m_routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(m_elevator::setShoulderVoltage,
            log->{
                log.motor("WoS Shoulder Motor")
                    .voltage(m_appliedVoltage.mut_replace(m_elevator.getShoulderVoltage(), Volts))
                    .angularPosition(m_shoulderPosition.mut_replace(m_elevator.getShoulderPosition(), Radians))
                    .angularVelocity(m_shoulderVelocity.mut_replace(m_elevator.getShoulderVelocity(), RadiansPerSecond));
            }, m_elevator)
        );
    }    
}