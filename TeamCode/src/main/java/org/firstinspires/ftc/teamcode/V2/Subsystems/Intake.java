package org.firstinspires.ftc.teamcode.V2.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.V2.Libs.Commands;

public class Intake extends SubsystemBase {

    //Hardware
    private final DcMotor motor1;
    private final DcMotor motor2;

    //Helpers
    private final Telemetry telemetry;

    public Intake(HardwareMap hw, Telemetry telemetry) {
        motor1 = hw.get(DcMotor.class, "IntakeMotor");
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        motor2 = hw.get(DcMotor.class, "Intake2Motor");
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        this.telemetry = telemetry;

    }

    public Command in() {
        return Commands.runEnd(
                () -> {
                        motor1.setPower(0.8);
                        motor2.setPower(0.8);
                },
        () -> {
                    motor1.setPower(0.0);
                  motor2.setPower(0.0);
        },
                this

        );
    }
    public Command out() {
        return Commands.runEnd(
                () -> {
                    motor1.setPower(-0.8);
                    motor2.setPower(-0.8);
                },
                () -> {
                    motor1.setPower(0.0);
                    motor2.setPower(0.0);
                },
this
        );

    }

}
