package frc.robot.SysId;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.WoS;

public class WosShoulderRoutine extends SubsystemRoutine {
    private WoS m_wos;

    private MutVoltage m_appliedVoltage = Volts.mutable(0);
    private MutAngle m_shoulderPosition = Radians.mutable(0);
    private MutAngularVelocity m_shoulderVelocity = RadiansPerSecond.mutable(0);

    public WosShoulderRoutine() {
        m_wos = WoS.getInstance();

        m_routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(m_wos::setShoulderVoltage,
            log->{
                log.motor("WoS Shoulder Motor")
                    .voltage(m_appliedVoltage.mut_replace(m_wos.getShoulderVoltage(), Volts))
                    .angularPosition(m_shoulderPosition.mut_replace(m_wos.getShoulderPosition(), Radians))
                    .angularVelocity(m_shoulderVelocity.mut_replace(m_wos.getShoulderVelocity(), RadiansPerSecond));
            }, m_wos)
        );
    }    
}