package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase{
    private final DriveIO io;
    private final DriveInputsAutoLogged inputs = new DriveInputsAutoLogged();

    private MecanumDriveKinematics kinematics;
    //private MecanumDriveWheelPositions positions;

    public Drive(DriveIO io) {
        this.io = io;

        Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
        Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
        Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
        Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

        //positions = new MecanumDriveWheelPositions(Meters.of(0), Meters.of(0), Meters.of(0), Meters.of(0));

        kinematics = new MecanumDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    private void runVelocity(ChassisSpeeds speeds){
        ChassisSpeeds discreteSpeeds= ChassisSpeeds.discretize(speeds, 0.02);
        MecanumDriveWheelSpeeds wheelSpeeds= kinematics.toWheelSpeeds(discreteSpeeds);

        wheelSpeeds.desaturate(7);
        
        io.setFRVelocity(RadiansPerSecond.of(wheelSpeeds.frontRightMetersPerSecond));
        io.setFLVelocity(RadiansPerSecond.of(wheelSpeeds.frontLeftMetersPerSecond));
        io.setRLVelocity(RadiansPerSecond.of(wheelSpeeds.rearLeftMetersPerSecond));
        io.setRRVelocity(RadiansPerSecond.of(wheelSpeeds.rearRightMetersPerSecond));
    }

    private void stop() {
        io.setFRVelocity(RadiansPerSecond.of(0));
        io.setFLVelocity(RadiansPerSecond.of(0));
        io.setRLVelocity(RadiansPerSecond.of(0));
        io.setRRVelocity(RadiansPerSecond.of(0));
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), .1);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        linearMagnitude = linearMagnitude * linearMagnitude;

        return new Pose2d(Translation2d.kZero, linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
            .getTranslation();
    }

    public Command joystickDrive(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier) {
        
        return drive.runEnd(
            () -> {
                // Get linear velocity
                Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                // Apply rotation deadband
                double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), .1);

                // Convert to field relative speeds & send command
                ChassisSpeeds speeds = new ChassisSpeeds(
                    linearVelocity.getX() * 7,
                    linearVelocity.getY() * 7,
                    omega * Math.hypot(.381,.381)
                );

                drive.runVelocity(speeds);
            },
            drive::stop
        );
    }

    public Command driveDistance(Drive drive, double distance, double time) {
        PIDController distancePID = new PIDController(1, 0, 0);
        io.resetEncoders();
        distancePID.setSetpoint(distance);
        distancePID.setTolerance(0.5);

        return drive.runEnd(
            () -> {
                double speed = distancePID.calculate(((inputs.flPosition).baseUnitMagnitude() + 
                    (inputs.frPosition).baseUnitMagnitude() +
                    (inputs.rlPosition).baseUnitMagnitude() +
                    (inputs.rrPosition).baseUnitMagnitude()) / 4.0);

                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speed, 0, 0);
                runVelocity(chassisSpeeds);
            },
            () -> {
                distancePID.close();
                drive.stop();
            }
        ).withTimeout(time);
    }

    public Command turn(double speed) {
        return run(
            () -> {
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, speed);
                runVelocity(chassisSpeeds);
            }
        );
    }
}
