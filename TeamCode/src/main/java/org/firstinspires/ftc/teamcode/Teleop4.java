package org.firstinspires.ftc.teamcode;


import com.bylazar.gamepad.PanelsGamepad;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

import lombok.val;


@TeleOp(name = "Teleop4")
public class Teleop4 extends LinearOpMode {
    //gamepad1
    private DcMotor lfMotor;
    private DcMotor lbMotor;
    private DcMotor rfMotor;
    private DcMotor rbMotor;
    //gamepad2

    // 4000 Shooter Revs   1 Motor Revs      1 Min        28 Ticks     1,867 Ticks
    // ----------------- * --------------- *  ---------- * ---------- = ---------
    // 1 Minutes           22/12 Shooter Revs    60 Seconds   1 Motors Rev   1 Second

    private DcMotorEx stMotor;
    private DcMotor ltMotor;
    private DcMotor inMotor;
    private DcMotor jkMotor;
    private CRServo hdMotor;
    private CRServo in2Motor;
    private Limelight3A limelight;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    GoBildaPinpointDriver odo;

        @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            lfMotor = hardwareMap.get(DcMotor.class, "frontleft");
            lbMotor = hardwareMap.get(DcMotor.class, "backleft");
            rfMotor = hardwareMap.get(DcMotor.class, "frontright");
            rbMotor = hardwareMap.get(DcMotor.class, "backright");
            stMotor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
            ltMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
            jkMotor = hardwareMap.get(DcMotor.class, "JackMotor");
            hdMotor = hardwareMap.get(CRServo.class, "HoodMotor");
            inMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
            in2Motor = hardwareMap.get(CRServo.class, "Intake2Motor");
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
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
        ltMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        jkMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();


        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                panelsTelemetry.debug("tx", result.getTx());
                panelsTelemetry.debug("ty", result.getTy());
                panelsTelemetry.debug("ta", result.getTa());
                panelsTelemetry.debug("Botpose", botpose.toString());
                panelsTelemetry.debug("AngularVelocity", stMotor.getVelocity());
                panelsTelemetry.update(telemetry);

            } else {
                Pose2D pos = odo.getPosition();
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.RADIANS));
                panelsTelemetry.debug("Position", data);
                panelsTelemetry.debug("RPM", stMotor.getVelocity());

                String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
                panelsTelemetry.debug("Velocity", velocity);

                panelsTelemetry.debug("Status", odo.getDeviceStatus());

                panelsTelemetry.debug("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

                panelsTelemetry.update(telemetry);
            }
            odo.update();

            gamepad1.left_stick_y *= Math.abs(gamepad1.left_stick_y);
            gamepad1.left_stick_x *= Math.abs(gamepad1.left_stick_x);

            gamepad1.right_stick_x *= Math.abs(gamepad1.right_stick_x);
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double denominater;
            denominater = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(x) + Math.abs(turn), 1);

            lfMotor.setPower((y + x + turn) / denominater);
            lbMotor.setPower((y - x + turn) / denominater);
            rfMotor.setPower((y - x - turn) / denominater);

            rbMotor.setPower((y + x - turn) / denominater);


            ltMotor.setPower(gamepad2.right_stick_y);
            jkMotor.setPower(gamepad2.left_stick_y);

            if (gamepad1.right_bumper) {
                inMotor.setPower(1);
            } else {
                inMotor.setPower(-1);
            }
            if (result.getTa() < 0.4) {
                if (gamepad2.left_bumper) {
                    stMotor.setVelocity(1548.3185);
                }
                else if (gamepad2.right_trigger > 0.4) {
                    stMotor.setPower(-0.25);
                } else {
                    stMotor.setPower(0);
                }
            }
            if (result.getTa() > 0.4) {
                if (gamepad2.left_bumper) {
                    stMotor.setVelocity(802.879);
                } else if (gamepad2.right_trigger > 0.4) {
                    stMotor.setPower(-0.25);
                } else {
                    stMotor.setPower(0);
                }
            }

                if (gamepad1.x) {
                    in2Motor.setPower(-1);
                } else if (gamepad1.y) {
                    in2Motor.setPower(1);
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
            }
            if (gamepad2.x) {
                in2Motor.setPower(1);
            } else if (gamepad2.y) {
                in2Motor.setPower(1);
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

            panelsTelemetry.debug("Status", "Initialized");
            panelsTelemetry.debug("X offset", odo.getXOffset(DistanceUnit.INCH));
            panelsTelemetry.debug("Y offset", odo.getYOffset(DistanceUnit.INCH));
            panelsTelemetry.debug("Device Version Number:", odo.getDeviceVersion());
            panelsTelemetry.debug("Heading Scalar", odo.getYawScalar());
            panelsTelemetry.debug("AngularVelocity", stMotor.getVelocity());
            panelsTelemetry.update(telemetry);



        }
    }



