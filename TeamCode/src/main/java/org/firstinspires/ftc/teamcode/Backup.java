package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Backup")
public class Backup extends LinearOpMode {
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
    private CRServo   hdMotor;
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


            lfMotor.setDirection(DcMotor.Direction.REVERSE);
            lbMotor.setDirection(DcMotor.Direction.FORWARD);
            rfMotor.setDirection(DcMotor.Direction.FORWARD);
            rbMotor.setDirection(DcMotor.Direction.REVERSE);

            lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            stMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
        {
           ElapsedTime runtime = new ElapsedTime();
            double targetTimeSeconds = 1.0;
            while (opModeIsActive() && runtime.seconds() < targetTimeSeconds) {
                lfMotor.setPower(-1);
                lbMotor.setPower(1);
                rfMotor.setPower(1);
                rbMotor.setPower(-1);
            }
            while (!opModeIsActive() && runtime.seconds() < targetTimeSeconds) {
            lfMotor.setPower(0);
            lbMotor.setPower(0);
            rfMotor.setPower(0);
            rbMotor.setPower(0);
            }
            }
        }
    }

