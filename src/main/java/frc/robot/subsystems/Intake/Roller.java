package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {
    private final RollerIO io;
    private final RollerInputsAutoLogged inputs = new RollerInputsAutoLogged();

    public Roller(RollerIO io){
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    private void runVelocitySetpoint(AngularVelocity velocity) {
        io.runVelocity(velocity);
    }

    public Command runRollers(AngularVelocity velocity) {
        return startEnd(()-> runVelocitySetpoint(velocity), () -> io.stop());
    }
}
