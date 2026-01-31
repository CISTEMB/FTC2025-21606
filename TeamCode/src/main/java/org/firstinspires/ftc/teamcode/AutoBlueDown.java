package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.nio.file.Paths;

@Autonomous(name = "AutoBlueDown")
public class AutoBlueDown extends LinearOpMode {


    private Limelight3A limelight;
    public TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private final Pose Target_Location = new Pose(72, 78);
    GoBildaPinpointDriver odo;
    private DcMotor jkMotor;
    private DcMotor ltMotor;
    private DcMotor lfMotor;
    private DcMotor lbMotor;
    private DcMotor rfMotor;
    private DcMotor rbMotor;
    private DcMotorEx stMotor;
    private CRServo in2Motor;
    private DcMotor inMotor;
    private CRServo hdMotor;
    ElapsedTime runtime;
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

            limelight.pipelineSwitch(0);

            /*
             * Starts polling for data.
             */
            limelight.start();
            odo.setOffsets(0, 0, DistanceUnit.MM);


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
        }
        while (opModeIsActive()) { {
                panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

            Follower follower = Constants.createFollower(hardwareMap);
                follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

                //paths = new Paths(follower); // Build paths

                panelsTelemetry.debug("Status", "Initialized");
                panelsTelemetry.update(telemetry);


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
                LLResult result = limelight.getLatestResult();
                Pose2D get2;

                //follower.setPose(Pose2d);
                // Log values to Panels and Driver Station
                panelsTelemetry.debug("Path State", pathState);
                panelsTelemetry.debug("X", follower.getPose().getX());
                panelsTelemetry.debug("Y", follower.getPose().getY());
                panelsTelemetry.debug("Heading", follower.getPose().getHeading());
                panelsTelemetry.update(telemetry);

            }


             class Paths {
                public PathChain Path1;

                 public Paths(Follower follower) {
                    Path1 = follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(56,36),

                                            new Pose(72,78)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                            .build();
                    stMotor.setVelocity(802.879);


                     if (runtime.seconds() > 8) {
                        in2Motor.setPower(1);
                    }
                     PathChain Path2;

                     if (runtime.seconds() > 15) {
                         Path2 = follower.pathBuilder().addPath(
                                 new BezierLine(
                                         new Pose(72,78),

                                         new Pose(72,98)
                                 )
                         ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)).build();
                     }
                }
            }



            }
        }
    }

