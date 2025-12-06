package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Autotest")
public class Autotest extends LinearOpMode {
    //gamepad1
    private DcMotor lfMotor;
    private DcMotor lbMotor;
    private DcMotor rfMotor;
    private DcMotor rbMotor;
    //gamepad2
    private DcMotor stMotor;
    private DcMotor ltMotor;
    private DcMotor inMotor;
    private DcMotor jkMotor;
    private CRServo hdMotor;
    private Limelight3A limelight;
    GoBildaPinpointDriver odo;
    @Override

    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            lfMotor = hardwareMap.get(DcMotor.class, "frontleft");
            lbMotor = hardwareMap.get(DcMotor.class, "backleft");
            rfMotor = hardwareMap.get(DcMotor.class, "frontright");
            rbMotor = hardwareMap.get(DcMotor.class, "backright");
            stMotor = hardwareMap.get(DcMotor.class, "ShooterMotor");
            ltMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
            jkMotor = hardwareMap.get(DcMotor.class, "JackMotor");
            hdMotor = hardwareMap.get(CRServo.class, "HoodMotor");
            inMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
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
        stMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
                // OpMode loop
            }
        }
    }

