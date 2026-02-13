package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Backup2")
public class Backup2 extends LinearOpMode {
    //gamepad1
    private DcMotor lfMotor;
    private DcMotor lbMotor;
    private DcMotor rfMotor;
    private DcMotor rbMotor;
    //gamepad2
    private DcMotor stMotor;
    private CRServo in2Motor;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (opModeIsActive()) {
            lfMotor = hardwareMap.get(DcMotor.class, "front-left");
            lbMotor = hardwareMap.get(DcMotor.class, "back-left");
            rfMotor = hardwareMap.get(DcMotor.class, "front-right");
            rbMotor = hardwareMap.get(DcMotor.class, "back-right");
            stMotor = hardwareMap.get(DcMotor.class, "ShooterMotor");
            DcMotor ltMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
            DcMotor jkMotor = hardwareMap.get(DcMotor.class, "JackMotor");
            CRServo hdMotor = hardwareMap.get(CRServo.class, "HoodMotor");
            DcMotor inMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
            in2Motor = hardwareMap.get(CRServo.class, "Intake2Motor");


            lfMotor.setDirection(DcMotor.Direction.REVERSE);
            lbMotor.setDirection(DcMotor.Direction.FORWARD);
            rfMotor.setDirection(DcMotor.Direction.FORWARD);
            rbMotor.setDirection(DcMotor.Direction.REVERSE);

            lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            stMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        {
           ElapsedTime runtime = new ElapsedTime();
            double targetTimeSeconds = .4;
            stMotor.setPower(0.7);
            while (opModeIsActive() && runtime.seconds() < targetTimeSeconds) {
                lfMotor.setPower(1);
                lbMotor.setPower(1);
                rfMotor.setPower(1);
                rbMotor.setPower(1);
                stMotor.setPower(0.7);
                in2Motor.setPower(1);
            }

            while (!opModeIsActive() && runtime.seconds() > targetTimeSeconds) {
            lfMotor.setPower(0);
            lbMotor.setPower(0);
            rfMotor.setPower(0);
            rbMotor.setPower(0);
            }
            }
        }
    }

