package org.firstinspires.ftc.teamcode.V2;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.RepeatCommand;

import org.firstinspires.ftc.teamcode.V2.RobotBase;
import org.firstinspires.ftc.teamcode.V2.Libs.Commands;
import org.firstinspires.ftc.teamcode.V2.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.V2.TeleOpV2;


public abstract class Auto extends RobotBase {
    static double startingpose;
    static double scoringHeading;
    private static Pose startPose;

    @TeleOp(group = "Blue")
    public static class AutoBlueUp extends TeleOpV2 {
        @Override
        public void initialize() {
            super.initialize();
            setBlueAlliance();
            startPose = new Pose(25, 125, Math.toRadians(90));
            scoringHeading = 135;
        }
    }

    @TeleOp(group = "Blue")
    public static class AutoBlueDown extends TeleOpV2 {
        @Override
        public void initialize() {
            super.initialize();
            setBlueAlliance();
            startPose = new Pose(56, 8, Math.toRadians(90));
            scoringHeading = 135;
        }
    }

    @TeleOp(group = "Red")
    public static class AutoRedUp extends TeleOpV2 {
        @Override
        public void initialize() {
            super.initialize();
            setRedAlliance();
            startPose = new Pose(120, 125, Math.toRadians(90));
            scoringHeading = 45;
        }
    }

    @TeleOp(group = "Red")
    public static class AutoRedDown extends TeleOpV2 {
        @Override
        public void initialize() {
            super.initialize();
            setRedAlliance();
            startPose = new Pose(90, 8, Math.toRadians(90));
            scoringHeading = 45;
        }
    }
    @Override
    public void run(){
        super.run();
        TeleOpV2.startingPose = drive.getFollower().getPose();
    }

    @Override
    protected void configureButtonBindings() {
        PathChain path1 = drive.getFollower().pathBuilder()
                .addPath(new BezierLine(
                        startPose,
                        new Pose(72, 72, Math.toRadians(scoringHeading)))
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();


        schedule(Commands.sequence(
                drive.followPath(path1),
                new RepeatCommand(visionShoot(), 3)
                ));

    }
}