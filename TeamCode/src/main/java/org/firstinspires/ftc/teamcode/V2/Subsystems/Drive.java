package org.firstinspires.ftc.teamcode.V2.Subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.V2.Libs.Commands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

public class Drive extends SubsystemBase {
    // Field Centric Constants

    //Hardware
    private final Follower follower;
    private Telemetry telemetry;
    public Follower getFollower(){
        return follower;
    }
    private double headingOffset_Rad;
    public void setHeadingOffset(double radians){
        headingOffset_Rad = radians;

    }
    public Drive(HardwareMap hw, Telemetry telemetry){
        follower = Constants.createFollower(hw);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public void arcadeDrive(double foward, double turn, double strafe)
    {

    }

    public Command setForward() {
        return Commands.runOnce(() -> follower.setHeading(Math.toRadians(90))
        );}
    public Command driveWithGamepad
            (Gamepad gamepad) {
        return new FunctionalCommand(
                //init
                () -> follower.startTeleOpDrive(),
                //excuete
                () -> {
                    double foward = -gamepad.left_stick_y;
                    double strafe = -gamepad.left_stick_x;
                    double turn = -gamepad.right_stick_x;


                    foward *= Math.abs(foward);
                    strafe *= Math.abs(strafe);
                    turn *= Math.abs(turn);


                    if (gamepad.right_trigger > 0.5) {
                        foward *= 0.25;
                        strafe *= 0.25;
                        turn *= 0.25;
                        telemetry.addData("Drive: SlowModeTrue", true);
                    } else {
                        telemetry.addData("Drive: SlowModeFalse", false);
                    }
                    telemetry.addData("Position", follower.getPose());
                    follower.setTeleOpDrive(foward, strafe, turn, false, headingOffset_Rad);
                },
                //end
        (interrupted) -> {
           follower.setTeleOpDrive(0, 0, 0);
        },
                ()-> false,
        this

        );
    }
}
