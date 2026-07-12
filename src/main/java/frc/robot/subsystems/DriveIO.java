package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface DriveIO {
    @AutoLog
    public static class DriveInputs {
        public Angle flPosition = Radians.zero();
        public AngularVelocity flVelocity = RadiansPerSecond.zero();
        public Voltage flVoltage = Volts.zero();
        public Current flCurrent = Amps.zero();

        public Angle rlPosition = Radians.zero();
        public AngularVelocity rlVelocity = RadiansPerSecond.zero();
        public Voltage rlVoltage = Volts.zero();
        public Current rlCurrent = Amps.zero();

        public Angle frPosition = Radians.zero();
        public AngularVelocity frVelocity = RadiansPerSecond.zero();
        public Voltage frVoltage = Volts.zero();
        public Current frCurrent = Amps.zero();

        public Angle rrPosition = Radians.zero();
        public AngularVelocity rrVelocity = RadiansPerSecond.zero();
        public Voltage rrVoltage = Volts.zero();
        public Current rrCurrent = Amps.zero();

    }

    public default void setVoltage(Voltage voltage) {};
    public default void setFLVelocity(AngularVelocity velocity) {};
    public default void setFRVelocity(AngularVelocity velocity) {};
    public default void setRLVelocity(AngularVelocity velocity) {};
    public default void setRRVelocity(AngularVelocity velocity) {};
    public default void resetEncoders() {};
    public default void updateInputs(DriveInputs inputs) {};
}