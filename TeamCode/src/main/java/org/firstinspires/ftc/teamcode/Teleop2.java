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




@TeleOp(name = "Teleop2")
public class Teleop2 extends LinearOpMode {
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
    GoBildaPinpointDriver odo;


        @Override
        public void runOpMode() throws InterruptedException {
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

                    telemetry.setMsTransmissionInterval(11);

                    limelight.pipelineSwitch(0);

                    /*
                     * Starts polling for data.
                     */
                    limelight.start();
                    odo.setOffsets(0, 0, DistanceUnit.MM);

                }

                lfMotor.setDirection(DcMotor.Direction.FORWARD);
                lbMotor.setDirection(DcMotor.Direction.REVERSE);
                rfMotor.setDirection(DcMotor.Direction.REVERSE);
                rbMotor.setDirection(DcMotor.Direction.FORWARD);
                ltMotor.setDirection(DcMotor.Direction.REVERSE);
                jkMotor.setDirection(DcMotor.Direction.FORWARD);

                lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                stMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
                odo.resetPosAndIMU();
                telemetry.addData("Status", "Initialized");
                telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
                telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
                telemetry.addData("Device Version Number:", odo.getDeviceVersion());
                telemetry.addData("Heading Scalar", odo.getYawScalar());
                telemetry.update();


                while (opModeIsActive()) {

                    gamepad1.left_stick_y = gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
                    gamepad1.left_stick_x = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);

                    gamepad1.right_stick_x = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
                    double y = -gamepad1.left_stick_y;
                    double x = gamepad1.left_stick_x;
                    double oldTime = 0;
                    double turn = gamepad1.right_stick_x;

                    double denominater = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(x) + Math.abs(turn), 1);

                    lfMotor.setPower((y + x + turn) / denominater);
                    lbMotor.setPower((y - x + turn) / denominater);
                    rfMotor.setPower((y - x - turn) / denominater);
                    rbMotor.setPower((y + x - turn) / denominater);

                    gamepad2.left_stick_y = gamepad2.left_stick_y * gamepad1.left_stick_y;

                    double lift = -gamepad2.right_stick_y;
                    double shoot = gamepad2.left_stick_x;


                    ltMotor.setPower(lift);
                    jkMotor.setPower(lift);

                    stMotor.setPower(shoot);
                    inMotor.setPower(1);
                    if (gamepad2.y) {
                        stMotor.setPower(1);
                    }
                    if (!gamepad2.y) {
                        hdMotor.setPower(0);
                    }
                    if (gamepad2.x) {
                        in2Motor.setPower(-1);
                    }
                    if (!gamepad2.x) {
                        in2Motor.setPower(0);
                    }
                    if (gamepad2.dpad_down) {
                        hdMotor.setPower(-1);
                    }
                    if (!gamepad2.dpad_down) {
                        hdMotor.setPower(0);
                    }
                    if (gamepad2.dpad_up) {
                        hdMotor.setPower(1);
                    }
                    if (!gamepad2.dpad_up) {
                        hdMotor.setPower(0);
                    }

                    LLResult result = limelight.getLatestResult();
                    if (result != null) {
                        if (result.isValid()) {
                            Pose3D botpose = result.getBotpose();
                            telemetry.addData("tx", result.getTx());
                            telemetry.addData("ty", result.getTy());
                            telemetry.addData("Botpose", botpose.toString());
                            telemetry.update()
                            ;
                        }
                    }
                    odo.update();

                    double newTime = getRuntime();
                    double loopTime = newTime - oldTime;
                    double frequency = 1 / loopTime;
                    oldTime = newTime;


                    Pose2D pos = odo.getPosition();
                    String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
                    telemetry.addData("Position", data);

                    String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
                    telemetry.addData("Velocity", velocity);

                    telemetry.addData("Status", odo.getDeviceStatus());

                    telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

                    telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
                    telemetry.update();

                }
            }
        }

