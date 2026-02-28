package org.firstinspires.ftc.teamcode;


import static java.lang.Math.tan;

import com.arcrobotics.ftclib.util.InterpLUT;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Locale;


@TeleOp(name = "TeleopAuto")
public class TeleopAuto extends LinearOpMode {

    //AutoShoot Stages
    public enum AutoShootState {
        kIdle,
        kAlignWithTarget,
        kGrabShooterRPM,
        kWaitForShooterRPM,
        kWaitforShot,

    }
    //Declare Look Up Table
    InterpLUT RPMlut = new InterpLUT();
    //Constants
    public double kStP = 0.032;
    public double kStF = 0.002;
    public double kLLP = 0.03;
    public double kTestRPM = 3000;
    public  double kHdDown = 0;
    public  double kHdUP = 0.3; //5 teeth
    public  double kmaxShooterPercentError = 0.05;
    public double kvelocityDipPercent = 0.1;

    //Device Imports
    private DcMotor lfMotor;
    private DcMotor lbMotor;
    private VoltageSensor voltageSensor;
    private DcMotor rfMotor;
    private DcMotor rbMotor;
    private DcMotorEx stMotor;
    private DcMotorEx stMotor2;
    private DcMotor inMotor;
    private DcMotor in2Motor;
    private Servo hdMotor;
    private CRServo FeederMotor;

    private Limelight3A limelight;

    private GoBildaRGBIndicator leftRGB;
    private GoBildaRGBIndicator rightRGB;
    //declare Auto shoot Enum
    private AutoShootState autoShootState = AutoShootState.kIdle;
    //Variable Import
    double GoalRPM;
    private double shooterPercentError;
    private double LatchedLLDistance;
    GoBaldaPinpointDriver odo;
    //TelemetryManager set to Panels
    TelemetryManager panelsTelemetry;
    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
            //Device Hardware Mapping
            voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
            lfMotor = hardwareMap.get(DcMotor.class, "front-left");
            lbMotor = hardwareMap.get(DcMotor.class, "back-left");
            rfMotor = hardwareMap.get(DcMotor.class, "front-right");
            rbMotor = hardwareMap.get(DcMotor.class, "back-right");
            stMotor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
            stMotor2 = hardwareMap.get(DcMotorEx.class , "ShooterMotor2");
            hdMotor = hardwareMap.get(Servo.class, "HoodMotor");
            inMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
            in2Motor = hardwareMap.get(DcMotor.class, "Intake2Motor");
            FeederMotor = hardwareMap.get(CRServo.class, "FeederMotor");
            odo = hardwareMap.get(GoBaldaPinpointDriver.class, "pinpoint");
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            leftRGB = new GoBildaRGBIndicator(hardwareMap, "LeftRGB");
            rightRGB = new GoBildaRGBIndicator(hardwareMap, "RightRGB");
            panelsTelemetry.debug(11);

            limelight.start();
            limelight.pipelineSwitch(3);
            odo.setOffsets(4, 0, DistanceUnit.INCH);
            odo.setPosition(72, 98);


            //LUT Values
            RPMlut.add(-100,0);
            RPMlut.add(0, 0);
            RPMlut.add(23.2, 2400);
            RPMlut.add(29.5, 2550);
            RPMlut.add(41.4, 2750);
            RPMlut.add(52.7, 3000);
            RPMlut.add(58.6, 3100);
            RPMlut.add(65.3, 3215);
            RPMlut.add(71.6, 3300);
            RPMlut.add(77.8, 3375);
            RPMlut.add(80.0, 3375);
            RPMlut.add(81.0, 0);
            RPMlut.add(105, 0);
            RPMlut.add(110, 3515);
            RPMlut.add(135, 3555);
            RPMlut.add(1000,3555);

            RPMlut.createLUT();

        }

        //Motor Directions

        lfMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        lbMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rbMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        stMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        stMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        inMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        in2Motor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Motor Modes

        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        inMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        in2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Constants Setup

        odo.setEncoderResolution(GoBaldaPinpointDriver.GoBaldaOdometryPods.goBALDA_4_BAR_POD);
        odo.setEncoderDirections(GoBaldaPinpointDriver.EncoderDirection.FORWARD, GoBaldaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        hdMotor.setPosition(0);


        while (opModeIsActive()) {
            //Limelight Data
            LLResult result = limelight.getLatestResult();
            double h2 = 29.5;
            double h1 = 12.7127;
            double a2 = 21.9714;
            double a1 = result.getTy();
            double d = (h2 - h1) / tan((a1 + a2) * 0.017453292519943295);
            odo.update();
            if (gamepad2.back) {
                limelight.pipelineSwitch(0);
            } else if (gamepad2.start) {
                limelight.pipelineSwitch(1);
            }


            double pipeline = result.getPipelineIndex();
            if (result.isValid() && pipeline == 0) {
                leftRGB.set(GoBildaRGBIndicator.Color.Red);
                rightRGB.set(GoBildaRGBIndicator.Color.Red);
            } else if (result.isValid() && pipeline == 1) {
                leftRGB.set(GoBildaRGBIndicator.Color.Blue);
                rightRGB.set(GoBildaRGBIndicator.Color.Blue);
            } else {
                leftRGB.set(GoBildaRGBIndicator.Color.Off);
                rightRGB.set(GoBildaRGBIndicator.Color.Off);
            }

            //Turn Values
            double y = 0;
            double x = 0;
            double turn = 0;


                    //Auto Shoot
                    switch (autoShootState) {
                        case kIdle:
                            FeederMotor.setPower(0);
                            GoalRPM = 0;
                            hdMotor.setPosition(kHdDown);

                            if (gamepad2.a) {
                                autoShootState = AutoShootState.kAlignWithTarget;
                            }
                            break;

                        case kAlignWithTarget:
                            LatchedLLDistance = d;
                            if (LatchedLLDistance > 110) {
                                hdMotor.setPosition(kHdUP);
                            } else {
                                hdMotor.setPosition(kHdDown);
                            }
                            FeederMotor.setPower(0);

                            double tx = result.getTx();
                            turn = tx * kLLP;
                            if (Math.abs(tx) < 2.5 && result.isValid()) {
                                autoShootState = AutoShootState.kGrabShooterRPM;
                            }
                            break;
                        case kGrabShooterRPM:
                            LatchedLLDistance = d;

                            if (LatchedLLDistance > 110) {
                                hdMotor.setPosition(kHdUP);
                            } else {
                                hdMotor.setPosition(kHdDown);
                            }
                            FeederMotor.setPower(0);
                            tx = result.getTx();
                            turn = tx * kLLP;

                            if (0<=d && d<=150) {
                                GoalRPM = RPMlut.get(d);
                            } else {
                                GoalRPM = 0;
                            }

                            autoShootState = AutoShootState.kWaitForShooterRPM;
                            break;

                        case kWaitForShooterRPM:
                            tx = result.getTx();
                            turn = tx * kLLP;
                            FeederMotor.setPower(0);

                            if (Math.abs(shooterPercentError) < kmaxShooterPercentError) {
                                autoShootState = AutoShootState.kWaitforShot;
                            }
                            break;
                        case kWaitforShot:
                            tx = result.getTx();
                            turn = tx * kLLP;

                            if (LatchedLLDistance > 100) {
                                hdMotor.setPosition(kHdUP);
                            } else {
                                hdMotor.setPosition(kHdDown);
                            }
                            FeederMotor.setPower(1);
                            if (Math.abs(shooterPercentError) > kvelocityDipPercent) {
                                autoShootState = AutoShootState.kWaitForShooterRPM;
                            }

                            break;
                    }
            //Chasis driver Control
            if (!gamepad2.a) {
                autoShootState = AutoShootState.kIdle;

                //slow Mode
                if (gamepad1.right_trigger > 0.5){
                    gamepad1.left_stick_y *= Math.abs(gamepad1.left_stick_y) - 0.3;
                    gamepad1.left_stick_x *= Math.abs(gamepad1.left_stick_x) -0.3;
                    gamepad1.right_stick_x *= Math.abs(gamepad2.right_stick_x) - 0.3;
                }

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

            //Intake and shooter manual control
            if (gamepad2.right_bumper && autoShootState == AutoShootState.kIdle) {
                GoalRPM = -6000;
            }
            if (gamepad1.left_bumper) {
                inMotor.setPower(-0.5);
                in2Motor.setPower(-0.5);
            } else {
                inMotor.setPower(0.5);
                in2Motor.setPower(0.5);
            }
            if (gamepad2.left_bumper){
                GoalRPM = 3515;
            }
            if (gamepad2.dpad_up) {
                hdMotor.setPosition(kHdUP);
            } else if (gamepad2.dpad_down) {
                hdMotor.setPosition(kHdDown);
            }
            if (gamepad2.x){
                FeederMotor.setPower(1);
            }

            //AngularVelocity to RPM

            // 4000 Shooter Revs   1 Motor Revs      1 Min        28 Ticks     1,867 Ticks
            // ----------------- * --------------- *  ---------- * ---------- = ---------
            // 1 Minutes           22/12 Shooter Revs    60 Seconds   1 Motors Rev   1 Second

            //PID Controller
            double batteryVolt = voltageSensor.getVoltage();
            double EncoderRPM = stMotor.getVelocity() / 28 * 60 * (60.0 / 36.0);

            double FFVolts = kStF * GoalRPM;
            double pidError = GoalRPM - EncoderRPM;
            shooterPercentError = (GoalRPM - EncoderRPM) / GoalRPM;
            double pidVolts = 0;
            pidVolts += pidVolts + kStP * pidError;
            ;

            double outputVolt = FFVolts + pidVolts;
            double outputPercent = outputVolt / batteryVolt;
            stMotor.setPower(outputPercent);
            stMotor2.setPower(outputPercent);

        //LimeLight Telemetry

            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                panelsTelemetry.debug("Distance", d);
                panelsTelemetry.debug("tx", result.getTx());
                panelsTelemetry.debug("ty", result.getTy());
                panelsTelemetry.debug("Bot pose", botpose.toString());
            }

        //All Time Telemetry
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            panelsTelemetry.debug("Pipeline", pipeline);
            panelsTelemetry.debug("Result", result.isValid());
            panelsTelemetry.debug("Odometry Position" , data);
            panelsTelemetry.debug("Heading Scalar", odo.getYawScalar());
            panelsTelemetry.addData("GoalRPM", GoalRPM);
            panelsTelemetry.addData("EncoderRPM", EncoderRPM);
            panelsTelemetry.addData("OutputVolts", outputVolt);
            panelsTelemetry.addData("ShooterPercentError", shooterPercentError);
            panelsTelemetry.addData("AutoShootState", autoShootState.toString());
            panelsTelemetry.update(telemetry);

        }


    }


}





