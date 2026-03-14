package org.firstinspires.ftc.teamcode.V2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.V2.Libs.CommandGamepad;

@TeleOp(name = "TeleOp V2")
public class TeleOpV2 extends RobotBase {

    @Override
    protected void configureButtonBindings() {
        //Defaults
        intake.setDefaultCommand(intake.in());

        //Gamepad 1
        CommandGamepad1.leftBumper().whileHeld(intake.out());

        //Gamepad 2
        CommandGamepad2.dpadDown().whenPressed(hood.down());
        CommandGamepad2.dpadUp().whenPressed(hood.up());

    }
}
