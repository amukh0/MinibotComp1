package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake.Roller;

public class AutoCommands {
    public static Command score(Drive drive, Roller roller) {
        return Commands.sequence(
            Commands.deadline(
                Commands.sequence(
                    drive.driveDistance(drive, -2, 2.5),
                    drive.turn(-10).withTimeout(1.25),
                    drive.driveDistance(drive, -5, 2.5),
                    drive.turn(10).withTimeout(.4)
                ),
                roller.runRollers(RPM.of(-20000))
            ),
            roller.runRollers(RPM.of(20000)).withTimeout(2),
            drive.turn(10).withTimeout(1),
            Commands.deadline(
                drive.driveDistance(drive, -5, 3),
                roller.runRollers(RPM.of(-20000000)))
        );
    }

    public static Command collect(Drive drive, Roller roller) {
        return Commands.sequence(
            Commands.waitSeconds(5),
            Commands.deadline(
                Commands.sequence(
                    drive.driveDistance(drive, -5, 4),
                    drive.turn(10).withTimeout(1.5),
                    drive.driveDistance(drive, -5, 1.5)
                ),
                roller.runRollers(RPM.of(-200000))
            )
        );
    }
}
