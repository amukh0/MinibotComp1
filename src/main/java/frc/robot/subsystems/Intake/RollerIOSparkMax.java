package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.*;

public class RollerIOSparkMax implements RollerIO {
    private static final int kRollerCanId = 5;

    private final SparkMax roller;
    private final RelativeEncoder rollerEncoder;
    private final SparkClosedLoopController pidRoller;

    private SparkMaxConfig config = new SparkMaxConfig();

    @SuppressWarnings("removal")
    public RollerIOSparkMax() {
        roller = new SparkMax(kRollerCanId, MotorType.kBrushed);
        rollerEncoder = roller.getEncoder();
        pidRoller = roller.getClosedLoopController();

        config.closedLoop
            .p(10)
            .i(0)
            .d(0);

        config.encoder
            .positionConversionFactor(.05*Math.PI)
            .velocityConversionFactor(Math.PI*.05/60);

        config.smartCurrentLimit(60);
        config.closedLoop.feedForward.kV(1000);

        roller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(RollerInputs inputs) {
        inputs.position = Rotations.of(rollerEncoder.getPosition());
        inputs.velocity = RPM.of(rollerEncoder.getVelocity());
        inputs.voltage = Volts.of(roller.getBusVoltage());
        inputs.current = Amps.of(roller.getOutputCurrent());
    }

    @Override
    public void runVelocity(AngularVelocity velocity) {
        pidRoller.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    }

    @Override
    public void stop() {
        roller.stopMotor();
    }
}
