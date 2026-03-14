package org.firstinspires.ftc.teamcode.v2.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.v2.lib.Commands;

import java.util.function.DoubleSupplier;

@Configurable
public class Shooter extends SubsystemBase {
    //
    // Constants
    //
    public static double kStP = 0.032;
    public static double kStF = 0.002;

    public static double kAtGoalPercentError = 0.05;
    public static double kvelocityDipPercent = 0.1;


    //
    // Hardware
    //
    private final DcMotorEx motorA;
    private final DcMotorEx motorB;
    private VoltageSensor voltageSensor;

    //
    // State
    //
    private double goalRPM;
    private double percentError;


    //
    // Helpers
    //
    private final Telemetry telemetry;

    public Shooter(HardwareMap hw, Telemetry telemetry) {
        motorA = hw.get(DcMotorEx.class, "ShooterMotor");
        motorA.setDirection(DcMotorSimple.Direction.FORWARD);
        motorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorB = hw.get(DcMotorEx.class, "ShooterMotor2");
        motorB.setDirection(DcMotorSimple.Direction.REVERSE); // TODO WHY
        motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        voltageSensor = hw.get(VoltageSensor.class, "Control Hub");

        this.telemetry = telemetry;
    }

    public double getPercentError() {
        return percentError;
    }

    public boolean isAtGoal() {
        return Math.abs(percentError) < kAtGoalPercentError;
    }

    public boolean hasShot() {
        return Math.abs(percentError) > kvelocityDipPercent;
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
        double encoderRPM = motorA.getVelocity() / 28 * 60 * (60.0 / 36.0);

        double ffVolts = kStF * goalRPM;
        double pidError = goalRPM - encoderRPM;
        percentError = (goalRPM - encoderRPM) / goalRPM;
        double pidVolts = 0;
        pidVolts += pidVolts + kStP * pidError;

        double outputVolt = ffVolts + pidVolts;
        double outputPercent = outputVolt / batteryVolt;
        motorA.setPower(outputPercent);
        motorB.setPower(outputPercent);

        telemetry.addData("Shooter Goal RPM", goalRPM);
        telemetry.addData("Shooter Encoder RPM", encoderRPM);
        telemetry.addData("Shooter Output Volts", outputVolt);
        telemetry.addData("Shooter Percent Error", percentError);
    }
}
