import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "auto1.1")

public class auto1 extends LinearOpMode{

        private DcMotor lfMotor;
        private DcMotor lbMotor;
        private DcMotor rfMotor;
        private DcMotor rbMotor;
        private DcMotor stMotor;
        private WebcamName Webcam;
        AprilTagProcessor tagProcessor;



    @Override
        public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            lfMotor = hardwareMap.get(DcMotor.class, "frontleft");
            lbMotor = hardwareMap.get(DcMotor.class, "backleft");
            rfMotor = hardwareMap.get(DcMotor.class, "frontright");
            rbMotor = hardwareMap.get(DcMotor.class, "backright");
            stMotor = hardwareMap.get(DcMotor.class, "ShooterMotor");
           // Webcam = hardwareMap.get(WebcamName.class, "Webcam");


            lfMotor.setDirection(DcMotor.Direction.FORWARD);
            lbMotor.setDirection(DcMotor.Direction.REVERSE);
            rfMotor.setDirection(DcMotor.Direction.REVERSE);
            rbMotor.setDirection(DcMotor.Direction.FORWARD);

            lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            stMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

          //  tagProcessor = new AprilTagProcessor.Builder()
           //       .setDrawAxes(true)
            //        .setDrawCubeProjection(true)
                   //.setDrawTagID(true)
                    //.setDrawTagOutline(true)
                   // .build();
            //VisionPortal visionportal = new VisionPortal.Builder()
                    //.enableLiveView(true)
                    //.addProcessor(tagProcessor)
                    //.setCamera(Webcam)
                  //  .setCameraResolution(new Size(640, 480))
                //    .build();


        }
        while (opModeIsActive()) {
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("z", tag.ftcPose.z);
            }
            telemetry.update();





            /*

             * Proportional Integral Derivative Controller w/ Low pass filter and anti-windup

             */

           double Kp = 0;
           double Ki = 0;
           double Kd = 0;

            double reference = 0;
           double lastReference = reference;
             double integralSum = 0;

            double lastError = 0;

            double maxIntegralSum = 8;

            double a = 0.8; // a can be anything from 0 < a < 1
           double previousFilterEstimate = 0;
          double  currentFilterEstimate = 0;

            ElapsedTime timer = new ElapsedTime();


            //if (po) {



                double encoderPosition = lbMotor.getCurrentPosition();
                encoderPosition = lfMotor.getCurrentPosition();
                encoderPosition = rbMotor.getCurrentPosition();
                encoderPosition = rfMotor.getCurrentPosition();

               double error = reference - encoderPosition;

               double errorChange = (error - lastError);

                currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange;
                previousFilterEstimate = currentFilterEstimate;

                double derivative = currentFilterEstimate / timer.seconds();


                integralSum = integralSum + (error * timer.seconds());


                if (integralSum > maxIntegralSum) {
                    integralSum = maxIntegralSum;
                }

                if (integralSum < -maxIntegralSum) {
                    integralSum = -maxIntegralSum;
                }


                if (reference != lastReference) {
                    integralSum = 0;
                }

                double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                lbMotor.setPower(out);

                lastError = error;

            }
        }
    }
