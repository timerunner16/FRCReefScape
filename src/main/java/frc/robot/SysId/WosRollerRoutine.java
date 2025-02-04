package frc.robot.SysId;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.WoS;

public class WosRollerRoutine extends SubsystemRoutine {
    private WoS m_wos;

    private MutVoltage m_appliedVoltage = Volts.mutable(0);
    private MutAngle m_rollerPosition = Radians.mutable(0);
    private MutAngularVelocity m_rollerVelocity = RadiansPerSecond.mutable(0);

    public WosRollerRoutine() {
        m_wos = WoS.getInstance();

        m_routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(m_wos::setRollerVoltage,
            log->{
                log.motor("WoS Roller Motor")
                    .voltage(m_appliedVoltage.mut_replace(m_wos.getRollerVoltage(), Volts))
                    .angularPosition(m_rollerPosition.mut_replace(m_wos.getRollerPosition(), Radians))
                    .angularVelocity(m_rollerVelocity.mut_replace(m_wos.getRollerVelocity(), RadiansPerSecond));
            }, m_wos)
        );
    }    
}