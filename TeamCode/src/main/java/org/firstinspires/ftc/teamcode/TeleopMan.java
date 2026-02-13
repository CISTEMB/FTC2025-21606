package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;




@TeleOp(name = "TeleopMan")
public class TeleopMan extends LinearOpMode {
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
    GoBaldaPinpointDriver odo;


    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            lfMotor = hardwareMap.get(DcMotor.class, "front-left");
            lbMotor = hardwareMap.get(DcMotor.class, "back-left");
            rfMotor = hardwareMap.get(DcMotor.class, "front-right");
            rbMotor = hardwareMap.get(DcMotor.class, "back-right");
            stMotor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
            ltMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
            jkMotor = hardwareMap.get(DcMotor.class, "JackMotor");
            hdMotor = hardwareMap.get(CRServo.class, "HoodMotor");
            inMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
            in2Motor = hardwareMap.get(CRServo.class, "Intake2Motor");
            odo = hardwareMap.get(GoBaldaPinpointDriver.class, "pinpoint");
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            telemetry.setMsTransmissionInterval(11);

            limelight.start();
            odo.setOffsets(4, 0, DistanceUnit.INCH);

        }

        lfMotor.setDirection(DcMotor.Direction.FORWARD);
        lbMotor.setDirection(DcMotor.Direction.REVERSE);
        rfMotor.setDirection(DcMotor.Direction.REVERSE);
        rbMotor.setDirection(DcMotor.Direction.FORWARD);
        ltMotor.setDirection(DcMotor.Direction.FORWARD);
        jkMotor.setDirection(DcMotor.Direction.FORWARD);
        in2Motor.setDirection(DcMotor.Direction.REVERSE);


        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        odo.setEncoderResolution(GoBaldaPinpointDriver.GoBaldaOdometryPods.goBALDA_4_BAR_POD);
        odo.setEncoderDirections(GoBaldaPinpointDriver.EncoderDirection.FORWARD, GoBaldaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();


        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());
                telemetry.addData("Botpose", botpose.toString());
                telemetry.addData("RPM", stMotor.getVelocity());
                telemetry.update();

            } else {
                Pose2D pos = odo.getPosition();
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Position", data);
                telemetry.addData("RPM", stMotor.getVelocity());

                String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
                telemetry.addData("Velocity", velocity);

                telemetry.addData("Status", odo.getDeviceStatus());

                telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

                telemetry.update();
            }
            odo.update();

            gamepad1.left_stick_y = gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
            gamepad1.left_stick_x = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);

            gamepad1.right_stick_x = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double denominate = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(x) + Math.abs(turn), 1);

            lfMotor.setPower((y + x + turn) / denominate);
            lbMotor.setPower((y - x + turn) / denominate);
            rfMotor.setPower((y - x - turn) / denominate);

            rbMotor.setPower((y + x - turn) / denominate);

            ltMotor.setPower(gamepad2.right_stick_y);
            jkMotor.setPower(gamepad2.left_stick_y);

            if (!gamepad1.right_bumper) {
                inMotor.setPower(-1);
            } else if (gamepad1.right_bumper) {
                inMotor.setPower(1);
            } else {
                inMotor.setPower(0);
            }
            //if (result.getTa() < 0.4) {
            if (gamepad2.left_bumper) {
                stMotor.setVelocity(802.879);

            }
            else if (gamepad2.left_trigger >0.4) {
                stMotor.setVelocity(1548.3185);
            }

            else if (gamepad2.right_trigger > 0.4) {
                stMotor.setPower(-0.25);
            }
               else {
                    stMotor.setPower(0);
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

            telemetry.addData("Status", "Initialized");
            telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
            telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
            telemetry.addData("Device Version Number:", odo.getDeviceVersion());
            telemetry.addData("Heading Scalar", odo.getYawScalar());
            telemetry.addData("RPM", stMotor.getVelocity());
            telemetry.update();


    }
    }