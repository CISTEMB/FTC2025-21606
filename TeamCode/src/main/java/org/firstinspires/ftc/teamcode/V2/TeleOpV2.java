package org.firstinspires.ftc.teamcode.V2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.V2.Libs.Commands;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Vision;

@TeleOp(name="TeleOpV2")
public class TeleOpV2 extends RobotBase {

    @Override
    protected void configureButtonBindings() {

        // Defaults

        intake.setDefaultCommand(intake.in());
        drive.setDefaultCommand(drive.driveWithGamepad(gamepad1));


        // Gamepad 1
        CommandGamepad1.leftBumper().whileHeld(intake.out());


        // Gamepad 2

        CommandGamepad2.a().whileHeld(visionShoot2());

        CommandGamepad2.dpadUp().whenPressed(hood.up());
        CommandGamepad2.dpadDown().whenPressed(hood.down());

        CommandGamepad2.back().whenPressed(vision.setPipeline(Vision.Pipeline.kBlueOnly));
        CommandGamepad2.start().whenPressed(vision.setPipeline(Vision.Pipeline.kRedOnly));

        // Manual Shooter control
        CommandGamepad2.rightBumper().whileHeld(shooter.setRPM(-6000));
        CommandGamepad2.leftBumper().whileHeld(shooter.setRPM(3515));
        CommandGamepad2.x().whileHeld(feeder.in());
    }


}
