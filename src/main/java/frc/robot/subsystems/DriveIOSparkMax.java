package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
//import edu.wpi.first.wpilibj.BuiltInAccelerometer;
//import edu.wpi.first.wpilibj.BuiltInAccelerometer.Range;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class DriveIOSparkMax implements DriveIO{
    private static final int kFrontLeftCanId = 1;
    private static final int kRearLeftCanId = 2;
    private static final int kFrontRightCanId = 3;
    private static final int kRearRightCanId = 4;

    private final SparkMax frontLeft;
    private final SparkMax frontRight;
    private final SparkMax rearLeft;
    private final SparkMax rearRight;

    private final RelativeEncoder flEncoder;
    private final RelativeEncoder frEncoder;
    private final RelativeEncoder rlEncoder;
    private final RelativeEncoder rrEncoder;

    private final SparkClosedLoopController pidFL;
    private final SparkClosedLoopController pidFR;
    private final SparkClosedLoopController pidRL;
    private final SparkClosedLoopController pidRR;

    MecanumDrive robotDrive;

    private SparkMaxConfig configFLeft = new SparkMaxConfig();
    private SparkMaxConfig configFRight = new SparkMaxConfig();

    private SparkMaxConfig configRLeft = new SparkMaxConfig();
    private SparkMaxConfig configRRight = new SparkMaxConfig();

    //private final BuiltInAccelerometer accel = new BuiltInAccelerometer();

    @SuppressWarnings("removal")
    public DriveIOSparkMax() {
        frontLeft = new SparkMax(kFrontLeftCanId, MotorType.kBrushed);
        rearLeft = new SparkMax(kRearLeftCanId, MotorType.kBrushed);
        frontRight = new SparkMax(kFrontRightCanId, MotorType.kBrushed);
        rearRight = new SparkMax(kRearRightCanId, MotorType.kBrushed);

        flEncoder = frontLeft.getEncoder();
        frEncoder = frontRight.getEncoder();
        rlEncoder = rearLeft.getEncoder();
        rrEncoder = rearRight.getEncoder();

        pidFL = frontLeft.getClosedLoopController();
        pidFR = frontRight.getClosedLoopController();
        pidRL = rearLeft.getClosedLoopController();
        pidRR = rearRight.getClosedLoopController();

        configFLeft.closedLoop
            .p(1)
            .i(0)
            .d(0);

        configFLeft.encoder
            .positionConversionFactor(.1*Math.PI)
            .velocityConversionFactor(Math.PI*.1/60);

        configFLeft.smartCurrentLimit(60);
        configFLeft.closedLoop.feedForward.kV(1000000);


        configRLeft.closedLoop
            .p(1)
            .i(0)
            .d(0);

        configRLeft.encoder
            .positionConversionFactor(.1*Math.PI)
            .velocityConversionFactor(Math.PI*.1/60);

        configRLeft.smartCurrentLimit(60);
        configRLeft.closedLoop.feedForward.kV(1000000);
        configRLeft.inverted(true);


        configFRight.closedLoop
            .p(1)
            .i(0)
            .d(0);

        configFRight.encoder
            .positionConversionFactor(.1*Math.PI)
            .velocityConversionFactor(Math.PI*.1/60);

        //configRight.inverted(true);
        configFRight.smartCurrentLimit(60);
        configFRight.closedLoop.feedForward.kV(1000000);

        configRRight.closedLoop
            .p(1)
            .i(0)
            .d(0);

        configRRight.encoder
            .positionConversionFactor(.1*Math.PI)
            .velocityConversionFactor(Math.PI*.1/60);

        //configRight.inverted(true);
        configRRight.smartCurrentLimit(60);
        configRRight.closedLoop.feedForward.kV(1000000);

        frontLeft.configure(configFLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rearLeft.configure(configRLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        frontRight.configure(configFRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rearRight.configure(configRRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    }

    @Override
    public void updateInputs(DriveInputs inputs) {
        inputs.flPosition = Rotations.of(flEncoder.getPosition());
        inputs.flVelocity = RPM.of(flEncoder.getVelocity());
        inputs.flVoltage = Volts.of(frontLeft.getBusVoltage());
        inputs.flCurrent = Amps.of(frontLeft.getOutputCurrent());

        inputs.frPosition = Rotations.of(frEncoder.getPosition());
        inputs.frVelocity = RPM.of(frEncoder.getVelocity());
        inputs.frVoltage = Volts.of(frontRight.getBusVoltage());
        inputs.frCurrent = Amps.of(frontRight.getOutputCurrent());

        inputs.rlPosition = Rotations.of(rlEncoder.getPosition());
        inputs.rlVelocity = RPM.of(rlEncoder.getVelocity());
        inputs.rlVoltage = Volts.of(rearLeft.getBusVoltage());
        inputs.rlCurrent = Amps.of(rearLeft.getOutputCurrent());

        inputs.rrPosition = Rotations.of(rrEncoder.getPosition());
        inputs.rrVelocity = RPM.of(rrEncoder.getVelocity());
        inputs.rrVoltage = Volts.of(rearRight.getBusVoltage());
        inputs.rrCurrent = Amps.of(rearRight.getOutputCurrent());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        pidFL.setSetpoint(voltage.baseUnitMagnitude(), ControlType.kVoltage);
        pidFR.setSetpoint(voltage.baseUnitMagnitude(), ControlType.kVoltage);
        pidRL.setSetpoint(voltage.baseUnitMagnitude(), ControlType.kVoltage);
        pidRR.setSetpoint(voltage.baseUnitMagnitude(), ControlType.kVoltage);
    }

    @Override
    public void setFLVelocity(AngularVelocity velocity) {
        pidFL.setSetpoint(velocity.baseUnitMagnitude(), ControlType.kVelocity);
    }

    @Override
    public void setFRVelocity(AngularVelocity velocity) {
        pidFR.setSetpoint(velocity.baseUnitMagnitude(), ControlType.kVelocity);
    }

    @Override
    public void setRLVelocity(AngularVelocity velocity) {
        pidRL.setSetpoint(velocity.baseUnitMagnitude(), ControlType.kVelocity);
    }

    @Override
    public void setRRVelocity(AngularVelocity velocity) {
        pidRR.setSetpoint(velocity.baseUnitMagnitude(), ControlType.kVelocity);
    }

    @Override
    public void resetEncoders() {
        flEncoder.setPosition(0.0);
        frEncoder.setPosition(0.0);
        rlEncoder.setPosition(0.0);
        rrEncoder.setPosition(0.0);

    }
}


