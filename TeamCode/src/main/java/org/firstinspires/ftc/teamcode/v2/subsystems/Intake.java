package org.firstinspires.ftc.teamcode.v2.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.v2.lib.Commands;

public class Intake extends SubsystemBase {
    //
    // Hardware
    //
    private final DcMotor motorA;
    private final DcMotor motorB;

    //
    // Helpers
    //
    private final Telemetry telemetry;

    public Intake(HardwareMap hw, Telemetry telemetry) {
        motorA = hw.get(DcMotor.class, "IntakeMotor");
        motorA.setDirection(DcMotorSimple.Direction.FORWARD);
        motorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorB = hw.get(DcMotor.class, "Intake2Motor");
        motorB.setDirection(DcMotorSimple.Direction.FORWARD);
        motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetry = telemetry;
    }

    public Command in() {
        return Commands.runEnd(
                () -> {
                    motorA.setPower(0.8);
                    motorB.setPower(0.8);
                },
                () -> {
                    motorA.setPower(0.0);
                    motorB.setPower(0.0);
                },
                this
        );
    }

    public Command out() {
        return Commands.runEnd(
                () -> {
                    motorA.setPower(-0.5);
                    motorB.setPower(0.5);
                },
                () -> {
                    motorA.setPower(0.0);
                    motorB.setPower(0.0);
                },
                this
        );
    }
}
