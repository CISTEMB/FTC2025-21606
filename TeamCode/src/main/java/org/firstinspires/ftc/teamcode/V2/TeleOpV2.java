package org.firstinspires.ftc.teamcode.V2;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.V2.Libs.Commands;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Lights;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Vision;

public abstract class TeleOpV2 extends RobotBase {
    public static Pose startPose;

   @TeleOp(group = "Blue")
    public static class TeleOpBlue extends TeleOpV2 {
        @Override
        public void initialize() {
            super.initialize();
            setBlueAlliance();
            drive.getFollower().setPose(startPose == null ? new Pose() : startPose);
        }
    }
   @TeleOp(group = "Red")
    public static class TeleOpRed extends TeleOpV2 {
        @Override
        public void initialize() {
            super.initialize();
            setRedAlliance();
            drive.getFollower().setPose(startPose == null ? new Pose() : startPose);
        }
    }
    @Override
    protected void configureCommands() {

        // Defaults

        intake.setDefaultCommand(intake.in());
        drive.setDefaultCommand(drive.driveWithGamepad(gamepad1));
        lights.setDefaultCommand(lights.run(vision));
        shooter.setDefaultCommand(shooter.setRPM(2400));


        // Gamepad 1
        commandGamepad1.leftBumper().whileHeld(intake.out());


        commandGamepad1.back().whenPressed(drive.setForward());
        // Gamepad 2

        commandGamepad2.a().whileHeld(visionShoot());

        commandGamepad2.dpadUp().whenPressed(hood.up());
        commandGamepad2.dpadDown().whenPressed(hood.down());

        commandGamepad2.back().whenPressed(Commands.runOnce( ()-> setRedAlliance()));
        commandGamepad2.start().whenPressed(Commands.runOnce( ()-> setBlueAlliance()));

        // Manual Shooter control
        commandGamepad2.rightBumper().whileHeld(shooter.setRPM(-6000));
        commandGamepad2.leftBumper().whileHeld(shooter.setRPM(3515));
        commandGamepad2.x().whileHeld(feeder.in());
    }


}
