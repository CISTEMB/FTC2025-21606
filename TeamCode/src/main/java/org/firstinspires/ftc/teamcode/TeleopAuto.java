package org.firstinspires.ftc.teamcode;


import static java.lang.Math.tan;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Locale;


@TeleOp(name = "TeleopAuto")
public class TeleopAuto extends LinearOpMode {
    private DcMotor lfMotor;
    private DcMotor lbMotor;
    private VoltageSensor voltageSensor;
    private DcMotor rfMotor;
    private DcMotor rbMotor;

    // 4000 Shooter Revs   1 Motor Revs      1 Min        28 Ticks     1,867 Ticks
    // ----------------- * --------------- *  ---------- * ---------- = ---------
    // 1 Minutes           22/12 Shooter Revs    60 Seconds   1 Motors Rev   1 Second

    private DcMotorEx stMotor;
    private DcMotorEx stMotor2;
    private DcMotor inMotor;
    private CRServo hdMotor;
    private CRServo in2Motor;
    private Limelight3A limelight;
    private double GoalRPM = 0;

    TelemetryManager panelsTelemetry;
    GoBaldaPinpointDriver odo;

    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
            voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
            lfMotor = hardwareMap.get(DcMotor.class, "front-left");
            lbMotor = hardwareMap.get(DcMotor.class, "back-left");
            rfMotor = hardwareMap.get(DcMotor.class, "front-right");
            rbMotor = hardwareMap.get(DcMotor.class, "back-right");
            stMotor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
            stMotor2 = hardwareMap.get(DcMotorEx.class , "ShooterMotor2");
            hdMotor = hardwareMap.get(CRServo.class, "HoodMotor");
            inMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
            in2Motor = hardwareMap.get(CRServo.class, "Intake2Motor");

            odo = hardwareMap.get(GoBaldaPinpointDriver.class, "pinpoint");
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            panelsTelemetry.debug(11);

            limelight.start();
            odo.setOffsets(4, 0, DistanceUnit.INCH);
            odo.setPosition(72, 98);

        }

        lfMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        lbMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rbMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        stMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        stMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        inMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        odo.setEncoderResolution(GoBaldaPinpointDriver.GoBaldaOdometryPods.goBALDA_4_BAR_POD);
        odo.setEncoderDirections(GoBaldaPinpointDriver.EncoderDirection.FORWARD, GoBaldaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();


        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            odo.update();
            double y = 0;
            double x = 0;
            double turn = 0;
            if (gamepad1.a) {
                double tx = result.getTx();
                turn = tx * 0.1;

            } else {
                gamepad1.left_stick_y *= Math.abs(gamepad1.left_stick_y);
                gamepad1.left_stick_x *= Math.abs(gamepad1.left_stick_x);

                gamepad1.right_stick_x *= Math.abs(gamepad1.right_stick_x);
                y = -gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                turn = gamepad1.right_stick_x;
            }
            double denominate;
            denominate = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(x) + Math.abs(turn), 1);

            lfMotor.setPower((y + x + turn) / denominate);
            lbMotor.setPower((y - x + turn) / denominate);
            rfMotor.setPower((y - x - turn) / denominate);
            rbMotor.setPower((y + x - turn) / denominate);





            if (gamepad2.left_bumper) {
              //  GoalRPM = kTestRPM;
            } else if (gamepad2.right_bumper) {
                GoalRPM=-3000;
            } else {
                GoalRPM = 0;
            }

            if (gamepad2.x) {
                in2Motor.setPower(1);
            } else if (gamepad2.y) {
                in2Motor.setPower(-1);
            } else {
                in2Motor.setPower(0);
            }

            if (gamepad2.dpad_down) {
                hdMotor.setPower(-1);
            } else if (gamepad2.dpad_up) {
                hdMotor.setPower(1);
            } else {
                hdMotor.setPower(0);
            }

            if (gamepad2.x) {
                in2Motor.setPower(1);
            } else if (gamepad2.y) {
                in2Motor.setPower(-1);
            } else {
                in2Motor.setPower(0);
            }

            if (gamepad2.dpad_down) {
                hdMotor.setPower(-1);
            } else if (gamepad2.dpad_up) {
                hdMotor.setPower(1);
            } else {
                hdMotor.setPower(0);
            }
            if (gamepad1.right_bumper) {
                inMotor.setPower(0.5);
            } else {
                inMotor.setPower(-1);
            }

            double batteryVolt = voltageSensor.getVoltage();
            double EncoderRPM = stMotor.getVelocity() / 28 * 60 * (60.0 / 36.0);

            double FFVolts = 0.002 * GoalRPM;
            double pidError = GoalRPM - EncoderRPM;
            double pidVolts = 0;
            pidVolts += pidVolts + 0.032 * pidError;
            ;

            double outputVolt = FFVolts + pidVolts;
            double outputPercent = outputVolt / batteryVolt;
            stMotor.setPower(outputPercent);
            stMotor2.setPower(outputPercent);

            double h2 = 29.5;
            double h1 = 12.7127;
            double a2 = 21.9714;
            double a1 = result.getTy();
            double d = (h2 - h1) / tan((a1 + a2) * 0.017453292519943295);
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                panelsTelemetry.debug("Distance", d);
                panelsTelemetry.debug("tx", result.getTx());
                panelsTelemetry.debug("ty", result.getTy());
                panelsTelemetry.debug("Bot pose", botpose.toString());
            }
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            panelsTelemetry.debug("Odometry Position" , data);
            panelsTelemetry.debug("Heading Scalar", odo.getYawScalar());
            panelsTelemetry.addData("GoalRPM", GoalRPM);
            panelsTelemetry.addData("EncoderRPM", EncoderRPM);
            panelsTelemetry.addData("OutputVolts", outputVolt);
            panelsTelemetry.addData("Shooter Motor Current" , stMotor.getCurrent(CurrentUnit.AMPS));
            panelsTelemetry.update(telemetry);

        }


    }
}




