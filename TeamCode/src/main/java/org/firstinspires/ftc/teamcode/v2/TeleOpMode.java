package org.firstinspires.ftc.teamcode.v2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.v2.subsystems.Vision;

@TeleOp(name="TeleOp V2")
public class TeleOpMode extends RobotBase {

    @Override
    protected void configureButtonBindings() {
        //
        // Defaults
        //
        intake.setDefaultCommand(intake.in());
        drive.setDefaultCommand(drive.driveWithGamepad(gamepad1));

        //
        // Gamepad 1
        //
        commandGamepad1.leftBumper().whileHeld(intake.out());

        //
        // Gamepad 2
        //
        commandGamepad2.a().whileHeld(visionShoot2());

        commandGamepad2.dpadUp().whenPressed(hood.up());
        commandGamepad2.dpadDown().whenPressed(hood.down());

        commandGamepad2.back().whenPressed(vision.setPipeline(Vision.Pipeline.kBlueOnly));
        commandGamepad2.start().whenPressed(vision.setPipeline(Vision.Pipeline.kRedOnly));

        // Manual Shooter control
        commandGamepad2.rightBumper().whileHeld(shooter.setRPM(-6000));
        commandGamepad2.leftBumper().whileHeld(shooter.setRPM(3515));
        commandGamepad2.x().whileHeld(feeder.in());
    }
}
