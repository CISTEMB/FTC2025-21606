package org.firstinspires.ftc.teamcode.v2;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.v2.lib.Commands;
import org.firstinspires.ftc.teamcode.v2.subsystems.Vision;

@TeleOp(name="TeleOp V2")
public class TestAutoOpMode extends RobotBase {

    @Override
    protected void configureButtonBindings() {
        PathChain path1 = drive.getFollower().pathBuilder()
                .addPath(new BezierLine(
                        new Pose(0, 0, Math.toRadians(0)),
                        new Pose(16, 28, Math.toRadians(90)))
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();


        schedule(Commands.sequence(
                // DRIVE TO POSITION via pedo pathing
                visionShoot2(),
                visionShoot2(),
                visionShoot2(),

                // DRIVE TO somehwere else
                drive.followPath(path1)
        ));
    }
}
