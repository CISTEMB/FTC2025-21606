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

    }
}
