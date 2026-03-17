package org.firstinspires.ftc.teamcode.v2.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drive extends SubsystemBase {
    private final Follower follower;
    public Follower getFollower() {
        return follower;
    }

    public Drive(HardwareMap hw, Telemetry telemetry) {
        follower = Constants.createFollower(hw);
    }

    @Override
    public void periodic() {
        // Update pedopathing odometry, path following, and driving
        follower.update();
    }

    public void arcade(double forward, double turn, double strafe) {

    }

    public void stop() {

    }

    public Command robotCentricDrive(Gamepad gamepad) {
        return new FunctionalCommand(
            () -> follower.startTeleopDrive(),
            () -> {
                double forward = -gamepad.left_stick_y;
                double strafe = gamepad.left_stick_x;
                double turn = gamepad.right_stick_x;

                forward = Math.copySign(forward*forward, forward);
                strafe = Math.copySign(strafe*strafe, strafe);
                turn = Math.copySign(turn*turn, turn);

                // I haven't seen it done this way before!
                // Cool!
                // forward *= Math.abs(forward);
                // strafe *= Math.abs(strafe);
                // turn *= Math.abs(turn);

                if (gamepad.right_trigger > 0.5){
                    forward *= 0.5;
                    strafe *= 0.5;
                    turn *= 0.5;
                }

                follower.setTeleOpDrive(forward, strafe, turn, true);
            },
            (end) -> follower.setTeleOpDrive(0,0,0),
            () -> false,
            this
        );
    }

    public Command followPath(Path path) {
        return new FollowPathCommand(follower, path).addRequirements(this);
    }

    public Command followPath(PathChain pathChain) {
        return new FollowPathCommand(follower, pathChain).addRequirements(this);
    }

    public Command holdPoint(Pose pose, boolean isFieldCentric) {
        return new HoldPointCommand(follower, pose, isFieldCentric);
    }
}
