package org.firstinspires.ftc.teamcode.V2.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.V2.Libs.Commands;

public class Feeder extends SubsystemBase {

    //Hardware
    private final CRServo servo;

    //Helpers
    private final Telemetry telemetry;

    public Feeder(HardwareMap hw, Telemetry telemetry) {
        servo = hw.get(CRServo.class, "IntakeMotor");



        this.telemetry = telemetry;

    }

    public Command in() {
        return Commands.runEnd(
                () -> servo.setPower(1),
                () -> servo.setPower(0),
                this
        );
    }
    public Command out() {
        return Commands.runEnd(
                () -> servo.setPower(-1),
                () -> servo.setPower(0),
                this

        );
    }

}
