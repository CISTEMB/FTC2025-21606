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

public class Drive extends SubsystemBase {
    // Field Centric Constants
    double redForward = 0;
    double blueForward = Math.PI;
    double currentForward;
    double robotHeading_Rad;
    //Hardware
    private final Follower follower;
    private Follower pinpoint;

    public Follower getFollower(){
        return follower;
    }
    public Drive(HardwareMap hw, Telemetry telemetry){

        follower = Constants.createFollower(hw);


    }

    @Override
    public void periodic() {
        follower.update();
        pinpoint.update();
    }

    public void arcadeDrive(double foward, double turn, double strafe)
    {

    }

    public Command setForward() {
        return Commands.runOnce(() -> currentForward = robotHeading_Rad
        );}
    public Command driveWithGamepad(Gamepad gamepad) {
        return new FunctionalCommand(
                //init
                () -> follower.startTeleOpDrive(),
                //excuete
                () -> {
                    robotHeading_Rad = pinpoint.getHeading();
                    double foward = -gamepad.left_stick_y;
                    double strafe = -gamepad.left_stick_x;
                    double turn = -gamepad.right_stick_x;


                    foward *= Math.abs(foward);
                    strafe *= Math.abs(strafe);
                    turn *= Math.abs(turn);


                    if (gamepad.right_trigger > 0.5) {
                        foward *= 0.6;
                        strafe *= 0.6;
                        turn *= 0.6;
                    }

                    follower.setTeleOpDrive(foward, strafe, turn, false, currentForward);
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
