package org.firstinspires.ftc.teamcode.v2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp V2")
public class TeleOpMode extends RobotBase {

    @Override
    protected void configureButtonBindings() {
        //
        // Defaults
        //
        intake.setDefaultCommand(intake.in());

        //
        // Gamepad 1
        //
        commandGamepad1.leftBumper().whileHeld(intake.out());

        //
        // Gamepad 2
        //
        commandGamepad2.dpadUp().whenPressed(hood.up());
        commandGamepad2.dpadDown().whenPressed(hood.down());

        // Manual Shooter control
        commandGamepad2.rightBumper().whileHeld(shooter.setRPM(-6000));
        commandGamepad2.leftBumper().whileHeld(shooter.setRPM(3515));
        commandGamepad2.x().whileHeld(feeder.in());
    }
}
