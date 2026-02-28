package org.firstinspires.ftc.teamcode;

import static java.lang.Math.tan;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.nio.file.Paths;
import java.util.Locale;

@Autonomous(name = "AutoRedDown")
public class AutoRedDown extends LinearOpMode {

    public enum ShootState {
        kAlignWithTarget,
        kGrabShooterRPM,
        kWaitForShooterRPM,
        kWaitforShot,

    }

    public static double kStP = 0.032;
    public static double kStF = 0.002;
    public static double kLlF = 0.03;
    public static double kTestRPM = 3000;
    public static double kHdDown = 0;
    public static double kHdUP = 0.3; //5 teeth
    public static double kmaxShooterPercentError = 0.05;
    public static double kvelocityDipPercent = 0.1;
    public static double kIntakeSpeed = 1;
    private ShootState shootState = ShootState.kAlignWithTarget;
    private Limelight3A limelight;
    public TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private final Pose Target_Location = new Pose(72, 78);
//    GoBaldaPinpointDriver odo;


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
    private DcMotor in2Motor;
    private Servo hdMotor;
    private CRServo FeederMotor;
    private GoBildaRGBIndicator leftRGB;
    private GoBildaRGBIndicator rightRGB;
    private double GoalRPM;
    private double LatchedLLDistance;
    private double shooterPercentError;

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
            stMotor2 = hardwareMap.get(DcMotorEx.class, "ShooterMotor2");
            hdMotor = hardwareMap.get(Servo.class, "HoodMotor");
            inMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
            in2Motor = hardwareMap.get(DcMotor.class, "Intake2Motor");
            FeederMotor = hardwareMap.get(CRServo.class, "FeederMotor");
//            odo = hardwareMap.get(GoBaldaPinpointDriver.class, "pinpoint");
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            panelsTelemetry.debug(11);

            limelight.pipelineSwitch(3);

            /*
             * Starts polling for data.
             */
            limelight.start();
//            odo.setOffsets(0, 0, DistanceUnit.MM);


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


        }
        while (opModeIsActive()) {
            ElapsedTime runtime = new ElapsedTime();



            Follower follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

            // paths = new Paths(follower); // Build paths


            follower.update(); // Update Pedro Pathing


            boolean following = false;
            if (!following) {
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), Target_Location))
                                .setLinearHeadingInterpolation(follower.getHeading(), Target_Location.minus(follower.getPose()).getAsVector().getTheta())
                                .build()
                );
            }

            Pose2D get2;
            limelight.pipelineSwitch(3);




            class Paths {
                public PathChain Path1;

                public Paths(Follower follower) {
                    Path1 = follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(56, 36),

                                            new Pose(72, 98)
                                    )
                            ).setLinearHeadingInterpolation(Math.toDegrees(90), Math.toDegrees(180))

                            .build();
                }
            }
            LLResult result = limelight.getLatestResult();
            double h2 = 29.5;
            double h1 = 12.7127;
            double a2 = 21.9714;
            double a1 = result.getTy();
            double d = (h2 - h1) / tan((a1 + a2) * 0.017453292519943295);

            double y = 0;
            double x = 0;
            double turn = 0;

            // Paths Path1;
            switch (shootState) {
                case kAlignWithTarget:
                    hdMotor.setPosition(kHdUP);
                    FeederMotor.setPower(0);

                    double tx = result.getTx();
                    turn = tx * kLlF;
                    if (Math.abs(tx) < 2.5 && result.isValid()) {
                        shootState = ShootState.kGrabShooterRPM;
                    }
                    break;
                case kGrabShooterRPM:
                    hdMotor.setPosition(kHdUP);
                    FeederMotor.setPower(0);
                    tx = result.getTx();
                    turn = tx * kLlF;

                    GoalRPM = 3565;

                    shootState = ShootState.kWaitForShooterRPM;
                    break;
                case kWaitForShooterRPM:
                    tx = result.getTx();
                    turn = tx * kLlF;
                    hdMotor.setPosition(kHdUP);
                    FeederMotor.setPower(0);

                    if (Math.abs(shooterPercentError) < kmaxShooterPercentError) {
                        shootState = ShootState.kWaitforShot;
                    }
                    break;
                case kWaitforShot:
                    tx = result.getTx();
                    turn = tx * kLlF;
                    hdMotor.setPosition(kHdUP);
                    FeederMotor.setPower(1);
                    if (Math.abs(shooterPercentError) > kvelocityDipPercent) {
                        shootState = ShootState.kWaitForShooterRPM;
                    }


                    break;

            }
            if (runtime.seconds() > 28) {
                lfMotor.setPower(1);
                lbMotor.setPower(-1);
                rfMotor.setPower(-1);
                rbMotor.setPower(1);
            }


            double denominate;
            denominate = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(x) + Math.abs(turn), 1);

            lfMotor.setPower((y + x + turn) / denominate);
            lbMotor.setPower((y - x + turn) / denominate);
            rfMotor.setPower((y - x - turn) / denominate);
            rbMotor.setPower((y + x - turn) / denominate);
            //PID Controller
            double batteryVolt = voltageSensor.getVoltage();
            double EncoderRPM = stMotor.getVelocity() / 28 * 60 * (60.0 / 36.0);

            double FFVolts = kStF * GoalRPM;
            double pidError = GoalRPM - EncoderRPM;
            shooterPercentError = (GoalRPM - EncoderRPM) / GoalRPM;
            double pidVolts = 0;
            pidVolts += pidVolts + kStP * pidError;


            double outputVolt = FFVolts + pidVolts;
            double outputPercent = outputVolt / batteryVolt;

            in2Motor.setPower(0.5);
            inMotor.setPower(0.5);
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
            panelsTelemetry.debug("Result", result.isValid());
            panelsTelemetry.addData("GoalRPM", GoalRPM);
            panelsTelemetry.addData("EncoderRPM", EncoderRPM);
            panelsTelemetry.addData("OutputVolts", outputVolt);
            panelsTelemetry.addData("ShooterPercentError", shooterPercentError);


        }

    }
}
