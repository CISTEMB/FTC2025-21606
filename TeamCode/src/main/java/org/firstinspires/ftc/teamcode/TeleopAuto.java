package org.firstinspires.ftc.teamcode;


import static java.lang.Math.tan;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.lights.RGBIndicator;
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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Locale;


@TeleOp(name = "TeleopAuto")
public class TeleopAuto extends LinearOpMode {

    public enum AutoShootState {
        kIdle,
        kAlignWithTarget,
        kGrabShooterRPM,
        kWaitForShooterRPM,
        kWaitforShot,

    }

    //Configurables
    public double kStP = 0.032;
    public double kStF = 0.002;
    public double kLlF = 0.024;
    public double kTestRPM = 3000;
    public  double kHdDown = 0;
    public  double kHdUP = 0.3; //5 teeth
    public  double kmaxShooterPercentError = 0.01;
    public double kvelocityDipPercent = 0.01;
    private DcMotor lfMotor;
    private DcMotor lbMotor;
    private VoltageSensor voltageSensor;
    private DcMotor rfMotor;
    private DcMotor rbMotor;
    //AngularVelocity to RPM

    // 4000 Shooter Revs   1 Motor Revs      1 Min        28 Ticks     1,867 Ticks
    // ----------------- * --------------- *  ---------- * ---------- = ---------
    // 1 Minutes           22/12 Shooter Revs    60 Seconds   1 Motors Rev   1 Second

    //Device Imports
    private DcMotorEx stMotor;
    private DcMotorEx stMotor2;
    private DcMotor inMotor;
    private Servo hdMotor;
    private CRServo in2Motor;
    private Limelight3A limelight;
    private GoBildaRGBIndicator leftRGB;
    private GoBildaRGBIndicator rightRGB;
    private double GoalRPM = 0;
    private double shooterPercentError;
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
            in2Motor = hardwareMap.get(CRServo.class, "Intake2Motor");
            odo = hardwareMap.get(GoBaldaPinpointDriver.class, "pinpoint");
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            leftRGB = new GoBildaRGBIndicator(hardwareMap, "LeftRGB");
            rightRGB = new GoBildaRGBIndicator(hardwareMap, "RightRGB");
            panelsTelemetry.debug(11);

            limelight.start();
            odo.setOffsets(4, 0, DistanceUnit.INCH);
            odo.setPosition(72, 98);

        }

        //Motor Directions

        lfMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        lbMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rbMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        stMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        stMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        inMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Motor Modes

        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Constants Setup

        odo.setEncoderResolution(GoBaldaPinpointDriver.GoBaldaOdometryPods.goBALDA_4_BAR_POD);
        odo.setEncoderDirections(GoBaldaPinpointDriver.EncoderDirection.FORWARD, GoBaldaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        hdMotor.setPosition(0);


        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            limelight.pipelineSwitch(3);
            odo.update();
            if (result.isValid()) {
                leftRGB.set(GoBildaRGBIndicator.Color.Green);
                rightRGB.set(GoBildaRGBIndicator.Color.Green);
            } else {
                leftRGB.set(GoBildaRGBIndicator.Color.Off);
                rightRGB.set(GoBildaRGBIndicator.Color.Off);
            }


            double y = 0;
            double x = 0;
            double turn = 0;

            switch (autoShootState){
                case kIdle:
                    in2Motor.setPower(0);
                    GoalRPM = 0;
                    hdMotor.setPosition(kHdDown);

                    if (gamepad2.a){
                        autoShootState = AutoShootState.kAlignWithTarget;
                    }
                    break;
                case kAlignWithTarget:
                    hdMotor.setPosition(kHdUP);
                    in2Motor.setPower(0);

                    double tx = result.getTx();
                    turn = tx * kLlF;
                    if ( Math.abs(tx) < 2.5 && result.isValid()) {
                        autoShootState = AutoShootState.kGrabShooterRPM;
                    }
                    break;
                case kGrabShooterRPM:
                    hdMotor.setPosition(kHdUP);
                    in2Motor.setPower(0);
                    tx = result.getTx();
                    turn = tx * kLlF;

                    GoalRPM = 3515;

                    autoShootState = AutoShootState.kWaitForShooterRPM;
                    break;
                case kWaitForShooterRPM:
                    tx = result.getTx();
                    turn = tx * kLlF;
                    hdMotor.setPosition(kHdUP);
                    in2Motor.setPower(0);

                    if (Math.abs(shooterPercentError) < kmaxShooterPercentError){
                        autoShootState = AutoShootState.kWaitforShot;
                    }
                    break;
                case kWaitforShot:
                    tx = result.getTx();
                    turn = tx * kLlF;
                    hdMotor.setPosition(kHdUP);
                    in2Motor.setPower(1);
                    if (Math.abs(shooterPercentError) > kvelocityDipPercent) {
                        autoShootState = AutoShootState.kWaitForShooterRPM;
                    }

                    break;
            }

            if (!gamepad2.a) {
                autoShootState = AutoShootState.kIdle;

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

            if (!gamepad1.left_bumper) {
                inMotor.setPower(1);
            } else if (gamepad1.left_bumper) {
                inMotor.setPower(-1);
            } else {
                inMotor.setPower(0);
            }

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
            panelsTelemetry.addData("ShooterPercentError", shooterPercentError);
            panelsTelemetry.addData("AutoShootState", autoShootState.toString());
            panelsTelemetry.update(telemetry);

        }


    }

    private AutoShootState autoShootState = AutoShootState.kIdle;
}





