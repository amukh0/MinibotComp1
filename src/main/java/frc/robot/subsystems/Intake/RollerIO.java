package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public interface RollerIO {
    @AutoLog
    public static class RollerInputs {
        public Angle position = Radians.zero();
        public AngularVelocity velocity = RadiansPerSecond.zero();
        public Voltage voltage = Volts.zero();
        public Current current = Amps.zero();
    }

    public default void runVelocity(AngularVelocity velocity) {}
    public default void stop() {}
    public default void updateInputs(RollerInputs inputs) {}
}
