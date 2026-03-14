package org.firstinspires.ftc.teamcode.V2.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.V2.Libs.Commands;

import java.util.function.DoubleSupplier;

@Configurable
public class Shooter extends SubsystemBase {
    //Constants
    public double kStP = 0.032;
    public double kStF = 0.002;

    //Hardware
    private final DcMotorEx motor1;
    private final DcMotor motor2;
    private VoltageSensor voltageSensor;

    //State
    private double goalRPM;
    private double percentError;

    //Helpers
    private final Telemetry telemetry;

    public Shooter(HardwareMap hw, Telemetry telemetry) {
        motor1 = hw.get(DcMotorEx.class, "ShooterMotor");
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor2 = hw.get(DcMotor.class, "ShooterMotor2");
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        this.telemetry = telemetry;

    }
    public Command setRPM(double rpm) {
        return setRPM(() -> rpm);
    }

    public Command setRPM(DoubleSupplier rpm) {
        return Commands.runEnd(
                () -> goalRPM = rpm.getAsDouble(),
                () -> goalRPM = 0,
                this
        );

    }

    @Override
    public void periodic() {
        //AngularVelocity to RPM

        // 4000 Shooter Revs   1 Motor Revs      1 Min        28 Ticks     1,867 Ticks
        // ----------------- * --------------- *  ---------- * ---------- = ---------
        // 1 Minutes           22/12 Shooter Revs    60 Seconds   1 Motors Rev   1 Second

        //PID Controller
        double batteryVolt = voltageSensor.getVoltage();
        double encoderRPM = motor1.getVelocity() / 28 * 60 * (60.0 / 36.0);

        double ffVolts = kStF * goalRPM;
        double pidError = goalRPM - encoderRPM;
        percentError = (goalRPM - encoderRPM) / goalRPM;
        double pidVolts = 0;
        pidVolts += pidVolts + kStP * pidError;
        ;

        double outputVolt = ffVolts + pidVolts;
        double outputPercent = outputVolt / batteryVolt;
        motor1.setPower(outputPercent);
        motor2.setPower(outputPercent);

        telemetry.addData("Shooter GoalRPM", goalRPM);
        telemetry.addData("Shooter EncoderRPM", encoderRPM);
        telemetry.addData("Shooter OutputVolts", outputVolt);
        telemetry.addData("Shooter PercentError", percentError);
    }
}
