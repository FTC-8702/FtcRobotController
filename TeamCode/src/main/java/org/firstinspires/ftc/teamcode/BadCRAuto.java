package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Autonomous(name = "BadCRAuto", group = "Concept")
public class BadCRAuto extends LinearOpMode {
    // Variables for encoder drive
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;

    private DcMotor rightRear = null;

    private DcMotor outtakeMotor = null;
    private Servo Dropper = null;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();

    static final double COUNT_PER_REVOLUTION = 537.7; // Ticks per revolution

    static final double WHEEL_DIAMETER_INCH = 3.78;

    static final double GEAR_RATIO = 1.0;

    static final double COUNT_PER_INCH = (COUNT_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER_INCH * Math.PI); // conversion for ticks per inch
    static final double DRIVE_SPEED = 0.6; // Speed when moving forward
    static final double TURN_SPEED = 0.5; // Speed when turning

    static final double STRAFE_SPEED = 0.5; // Speed when strafe

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag; //variable stores instance of April Tag Processor
    private VisionPortal visionPortal; //Variable stores instance of the vision portal

    int myTagID; //Will store the int type for the id number

    @Override
    public void runOpMode() {
        initEncoderDrive();
        initDropper();
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();


        if (opModeIsActive()) {
            Dropper.setPosition(0.4);
            EncoderDrive(DRIVE_SPEED, 24,24,24,24,4.0); // Drive forward 24 inches
            EncoderDrive(TURN_SPEED, 24,-24,24,-24, 3.0); // turn left 90 degrees to scan Tag
            scanAprilTag();

            if(myTagID == 1)
            {
                telemetry.addLine("ALEX: The Path for Tag ID 1 will be started");
                EncoderDrive(DRIVE_SPEED, -26,-26,-26,-26,4.0);// Drive up to Backdrop
                EncoderDrive(STRAFE_SPEED, 5,-5,-5,5, 3.0);
                dropPixel();
                EncoderDrive(STRAFE_SPEED,25,-25,-25,25,4.0);
                EncoderDrive(DRIVE_SPEED,-7,-7,-7,-7,3.0);
            } else if(myTagID == 2) {
                telemetry.addLine("ALEX 2: The path for ID 2 will be started");
                EncoderDrive(DRIVE_SPEED, -27,-27,-27,-27,4.0);// Drive up to Backdrop
                EncoderDrive(STRAFE_SPEED, 2,-2,-2,2, 3.0);
                dropPixel();
                EncoderDrive(STRAFE_SPEED,30,-30,-30,30, 4);// Strafe right 12 inches
                EncoderDrive(DRIVE_SPEED,-7,-7,-7,-7,3.0);


            } else if(myTagID == 3) {
                telemetry.addLine("Alex 3: The path for ID 3 will be started");
                EncoderDrive(DRIVE_SPEED, -26,-26,-26,-26,4.0);// Drive up to Backdrop
                EncoderDrive(STRAFE_SPEED, -5,5,5,-5, 3.0);
                dropPixel();
                EncoderDrive(STRAFE_SPEED,40,-40,-40,40, 4);
                EncoderDrive(DRIVE_SPEED,-7,-7,-7,-7,3.0);

            } else {
                telemetry.addLine("The April Tag did not match 1,2,or 3");
                EncoderDrive(DRIVE_SPEED, -26,-26,-26,-26,4.0);// Drive up to Backdrop
                dropPixel();
                EncoderDrive(STRAFE_SPEED,30,-30,-30,30, 4);// Strafe right 12 inches
                EncoderDrive(DRIVE_SPEED,-7,-7,-7,-7,3.0);

            }
            telemetry.update();


            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.CM.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(640, 480)); // Choose a camera resolution. Not all cameras support all resolutions.
        builder.enableLiveView(true); // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG); // Set the stream format; MJPEG uses less bandwidth than default YUY2.

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        builder.addProcessor(aprilTag); // Set and enable the processor.
        visionPortal = builder.build();  // Build the Vision Portal, using the above settings.

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
    private void initEncoderDrive() {
        //Call Motors from control hub
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    } //end method initEncoderDrive()

    public void initDropper() {
        outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");
        Dropper = hardwareMap.get(Servo.class, "Dropper");
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ;
    } //end method initDropper


    /**
     * Add telemetry about AprilTag detections.
     */
    private int telemetryAprilTag() {

        int retVal = 100; //basic value if the size == 0

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        if(currentDetections.size() > 0)
        {
            retVal = currentDetections.get(0).id;
        }

        return retVal;

    }   // end method telemetryAprilTag()

    private int scanAprilTag()
    {

        for(int i = 0; i < 40; i++) //1 factor of 10 = checking for april tag for 1x10 = sec, etc
        {
            myTagID = telemetryAprilTag();
            telemetry.update(); // Push telemetry to the Driver Station.
            if((myTagID < 7) && (myTagID > 0)) //If tag found b4 counter is done, breaks
            {
                telemetry.addLine(String.format("\n Break out of for loop ==== (ID %d)", myTagID));
                telemetry.update();
                break;
            } // end if

            sleep(100); //short break
        } // end for

        return myTagID;

    }// end scan
    /**
     * Method EncoderDrive tell robot to move a set distance and completely stop once the timer ends
     * */
    private void EncoderDrive(double Speed, double leftFrontInches, double rightFrontInches,
                              double leftRearInches, double rightRearInches, double timeOut){
        int leftFrontTarget;
        int rightFrontTarget;
        int leftRearTarget;
        int rightRearTarget;

        leftFrontTarget = leftFront.getCurrentPosition() + (int) (leftFrontInches * COUNT_PER_INCH);
        rightFrontTarget = rightFront.getCurrentPosition() + (int) (rightFrontInches * COUNT_PER_INCH);
        leftRearTarget = leftRear.getCurrentPosition() + (int) (leftRearInches * COUNT_PER_INCH);
        rightRearTarget = rightRear.getCurrentPosition() + (int) (rightRearInches * COUNT_PER_INCH);

        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftRear.setTargetPosition(leftRearTarget);
        rightRear.setTargetPosition(rightRearTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(Math.abs(Speed));
        rightFront.setPower(Math.abs(Speed));
        leftRear.setPower(Math.abs(Speed));
        rightRear.setPower(Math.abs(Speed));

        runtime.reset();

        while (opModeIsActive() &&
                (runtime.seconds() < timeOut) &&
                (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Running to", " %7d :%7d", leftFrontTarget, rightFrontTarget, leftRearTarget, rightRearTarget);
            telemetry.addData("Currently at", " at %7d :%7d",
                    leftFront.getCurrentPosition(), rightFront.getCurrentPosition(),
                    leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
            telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);

    } //end EncoderDrive()

    public void dropPixel() {
        outtakeMotor.setTargetPosition(1200);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeMotor.setPower(0.75);

        while (opModeIsActive() && outtakeMotor.isBusy()) {
            telemetry.addData("Current height: ", outtakeMotor.getCurrentPosition());
            telemetry.update();
        }
        //wait for linear slides to go up then activate this code:

        Dropper.setPosition(0); // Drops pixel
        while (outtakeMotor.isBusy()) {
            telemetry.addData("Dropping", Dropper.getPosition());
            telemetry.update();
        }
        sleep(1000);

        EncoderDrive(DRIVE_SPEED, 3,3,3,3,4.0);// Bak up from back drop
        sleep(1500);

        Dropper.setPosition(0.58);
        outtakeMotor.setTargetPosition(5);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeMotor.setPower(0.75);

            sleep(250);

    } //end dropPixel()


}   // end class
