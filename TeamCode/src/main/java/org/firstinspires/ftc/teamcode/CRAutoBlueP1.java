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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


@Autonomous(name = "CRAutoBlueP1", group = "Concept")
public class CRAutoBlueP1 extends LinearOpMode {
    //encoder drive variables
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;

    private DcMotor rightRear = null;

    private DcMotor outtakeMotor = null;
    private Servo Dropper = null;

    private Servo spikeLift = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNT_PER_REVOLUTION = 537.7; // Ticks per revolution

    static final double WHEEL_DIAMETER_INCH = 3.78;

    static final double GEAR_RATIO = 1.0;

    static final double COUNT_PER_INCH = (COUNT_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER_INCH * Math.PI); // conversion for ticks per inch
    static final double DRIVE_SPEED = 0.6; // Speed when moving forward
    static final double TURN_SPEED = 0.5; // Speed when turning

    static final double STRAFE_SPEED = 0.5; // Speed when strafe


    //April Tag Variables
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag; //variable stores instance of April Tag Processor
    private VisionPortal visionPortal; //Variable stores instance of the vision portal

    int myTagID; //Will store the int type for the id number


    //TFOD Variables
    private static final String TFOD_MODEL_ASSET = "BlueMarker.tflite";
    private static final String[] LABELS = {"Blue Marker",};

    private TfodProcessor tfod; //The variable to store our instance of the TensorFlow Object Detection processor.

    private VisionPortal visionPortal2; //The variable to store our instance of the vision portal.

    float confidMin = 0.80f; //sets min confidence level
    int xSizeMin = 100; //sets min x width
    int xSizeMax = 175; //sets max x width
    int ySizeMin = 90; //sets min y height
    int ySizeMax = 160; //sets max y height
    int yMin = 200; //sets y coord min
    int yMax = 400; //sets y coord max
    int xMinSMR = 1; //sets min x coord for spike mark right
    int xMaxSMR = 300; //sets max x coord for spike mark right
    int xMinSMC = 301; //sets min x coord for spike mark center
    int xMaxSMC = 640; //sets max x coord for spike mark center
    int RightMark = 3;
    int CenterMark = 2;
    int LeftMark = 1;
    int SpikeMarkLocation = LeftMark;



    @Override
    public void runOpMode()
    {

        //TFOD op mode part
        initSpikeLift();
        initTfod();
        initEncoderDrive();
        initDropper();


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        Dropper.setPosition(0.4);


        if (opModeIsActive()) {
            int SpikeMarkLocationFinal = scanTFOD(); //for loop method where Spike Mark Location = telemetryTFOD();


            if (SpikeMarkLocationFinal == RightMark) {
                telemetry.addLine("Path for Spike Mark Right is starting");
                startPosition();
                EncoderDrive(DRIVE_SPEED, 1, 1, 1, 1, 4.0); // Forward
                EncoderDrive(STRAFE_SPEED, -1, 1, 1, -1, 4.0); // Strafe left
                spikeLift();
                EncoderDrive(STRAFE_SPEED, 4, -4, -4, 4, 4.0); // Strafe right
                EncoderDrive(DRIVE_SPEED, -3, -3, -3, -3, 4.0); // Backwards
                EncoderDrive(TURN_SPEED, -21, 21, -21, 21, 4.0); // Turn facing Tag
                myTagID = 1;


            } else if (SpikeMarkLocationFinal == CenterMark) {
                telemetry.addLine("Path for Spike Mark Center is starting");
                startPosition();
                EncoderDrive(DRIVE_SPEED,3,-3,3,-3,4.0); // rotate 180 degrees
                EncoderDrive(DRIVE_SPEED, -6, -6, -6, -6, 3.0); // Backward
                EncoderDrive(STRAFE_SPEED, -14, 14, 14, -14, 4.0); // Strafe left
                spikeLift();
                EncoderDrive(STRAFE_SPEED, 14, -14, -14, 14, 4.0); // Strafe right
                EncoderDrive(DRIVE_SPEED, 6, 6, 6, 6, 3.0); // Forwards
                EncoderDrive(TURN_SPEED, -21, 21, -21, 21, 4.0);// Turn facing tag
                myTagID = 2;

            } else {
                telemetry.addLine("Path for Spike Mark Left is starting");
                startPosition();
                EncoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 4.0); // forward
                EncoderDrive(STRAFE_SPEED, -25, 25, 25, -25, 4.0);// strafe left
                spikeLift();
                EncoderDrive(STRAFE_SPEED, 25, -25, -25, 25, 4.0); // strafe right
                EncoderDrive(DRIVE_SPEED, -3, -3, -3, -3, 4.0); // backward
                EncoderDrive(TURN_SPEED, -20, 20, -20, 20, 4.0); // turn facing tag
                myTagID = 3;


            }
            telemetry.update();

            //visionPortal2.close();


            //April Tag Portion of OP Mode
            // initAprilTag();

            //scanAprilTag();


            if (myTagID == 1) {
                telemetry.addLine("ALEX: The Path for Tag ID 1 will be started");
                EncoderDrive(DRIVE_SPEED, -18, -18, -18, -18, 4.0);// Drive up to Backdrop
                EncoderDrive(STRAFE_SPEED, 5, -5, -5, 5, 3.0); // strafe right
                dropPixel();
                EncoderDrive(STRAFE_SPEED, 25, -25, -25, 25, 4.0); // strafe right
                EncoderDrive(DRIVE_SPEED, -7, -7, -7, -7, 3.0); // park
            } else if (myTagID == 2) {
                telemetry.addLine("ALEX 2: The path for ID 2 will be started");
                telemetry.addLine("Approaching Backdrop");
                EncoderDrive(DRIVE_SPEED, -18, -18, -18, -18, 4.0);// Drive up to Backdrop
                EncoderDrive(STRAFE_SPEED, 9, -9, -9, 9, 3.0);// strafe right
                EncoderDrive(DRIVE_SPEED, -2, -2, -2, -2, 3.0); // go forward
                dropPixel();
                EncoderDrive(STRAFE_SPEED, 34, -34, -34, 34, 4);// Strafe right
                EncoderDrive(DRIVE_SPEED, -7, -7, -7, -7, 3.0); // park
            } else /*if (myTagID == 3)*/ {
                telemetry.addLine("Alex 3: The path for ID 3 will be started");
                telemetry.addLine("Approaching Backdrop");
                EncoderDrive(DRIVE_SPEED, -17, -17, -17, -17, 4.0);// Drive up to Backdrop
                EncoderDrive(STRAFE_SPEED, -8, 8, 8, -8, 3.0); // Strafe left
                EncoderDrive(DRIVE_SPEED, -3, -3, -3, -3, 3.0); // go forward
                dropPixel();
                EncoderDrive(STRAFE_SPEED, 40, -40, -40, 40, 4); // Strafe right
                EncoderDrive(DRIVE_SPEED, -7, -7, -7, -7, 3.0); // park
            }
            /*else {
                telemetry.addLine("The April Tag did not match 1,2,or 3");
                telemetry.addLine("Approaching Backdrop");
                EncoderDrive(DRIVE_SPEED, -17, -17, -17, -17, 4.0);// Drive up to Backdrop
                EncoderDrive(STRAFE_SPEED, 2, -2, -2, 2, 3.0);// strafe right
                dropPixel();
                EncoderDrive(STRAFE_SPEED, 30, -30, -30, 30, 4);// Strafe right
                EncoderDrive(DRIVE_SPEED, -7, -7, -7, -7, 3.0); // park

            }*/
            telemetry.update();
        }

            //visionPortal.close();

                // Share the CPU.
                sleep(20);
        }// end op mode

        // Save more CPU resources when camera is no longer needed.
        //visionPortal.close();

       // end method runOpMode()


    //APRIL TAG METHODS

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
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(640, 480)); // Choose a camera resolution. Not all cameras support all resolutions.

        builder.enableLiveView(true); // NEED TO SET TO FALSE FOR QUALIFIER OIJWEFOIJWEFOIJWOIEJFOIJWEF

        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG); // Set the stream format; MJPEG uses less bandwidth than default YUY2.

        builder.addProcessor(aprilTag); // Set and enable the processor.
        visionPortal = builder.build();  // Build the Vision Portal, using the above settings.


    }   // end method initAprilTag()


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
                //break;
                i=40;
            } // end if

            sleep(100); //short break
        } // end for

        return myTagID;

    }// end scan


    //TFOD METHODS

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2); // Set the stream format; MJPEG uses less bandwidth than default YUY2.

        builder.addProcessor(tfod); // Set and enable the processor.

        visionPortal2 = builder.build(); // Build the Vision Portal, using the above settings.

        tfod.setMinResultConfidence(confidMin); // Set confidence threshold for TFOD recognitions, at any time.

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private int telemetryTFOD()
    {
        int SpikeMarkLocationTelemetry = LeftMark;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());


            //first if makes sure Height of object is ySizeMax > Height > ySizeMin
            //second if makes sure Width of object is xSizeMax > Width > xSizeMin
            //third if makes sure object is between two y coordinates

            if ((((int) recognition.getHeight() < ySizeMax) && ((int) recognition.getHeight() > ySizeMin)) &&
                    (((int) recognition.getWidth() < xSizeMax) && ((int) recognition.getWidth() > xSizeMin)) &&
                    (((int) y < yMax) && ((int) y > yMin))) {
                if (((x < xMaxSMR) && (x > xMinSMR))) //Checks if marker is on right spike mark
                {
                    SpikeMarkLocationTelemetry = RightMark;
                    break;
                }
                if (((x < xMaxSMC) && (x > xMinSMC))) //checks if marker is on center marker, else its on the left one
                {
                    SpikeMarkLocationTelemetry = CenterMark;
                    break;
                }

            }//end big if
        }   // end for() loop

        return SpikeMarkLocationTelemetry;
    }


    //For loop for telemetry TFOD
    private int scanTFOD()
    {

        int SpikeMarkLocationScan = LeftMark;
        for(int i = 0; i < 40; i++) //1 factor of 10 = checking for april tag for 1x10 = sec, etc
        {
            SpikeMarkLocationScan = telemetryTFOD();
            telemetry.update(); // Push telemetry to the Driver Station.
            if( SpikeMarkLocationScan != LeftMark) //If tag found b4 counter is done, breaks
            {
                telemetry.addLine(String.format("\n Break out of for loop ==== (ID %d)", SpikeMarkLocation));
                telemetry.update();
                break;
            } // end if

            sleep(100); //short break
        } // end for

        return SpikeMarkLocationScan;

    }// end scan
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


        EncoderDrive(DRIVE_SPEED, .5,.5,.5,.5,4.0);// Bak up from back drop
        //EncoderDrive(DRIVE_SPEED, 1,1,1,1,4.0);// Bak up from back drop
        Dropper.setPosition(0.58);

        EncoderDrive(DRIVE_SPEED, 3,3,3,3,4.0);// Bak up from back drop
        sleep(200);



        outtakeMotor.setTargetPosition(5);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeMotor.setPower(0.75);

        sleep(250);

    } //end dropPixel()
    public void initSpikeLift() {
        spikeLift = hardwareMap.get(Servo.class,"spikeLift");
        spikeLift.setPosition(.6);
    }

    public void spikeLift() {
        spikeLift.setPosition(1);


    }
    public void startPosition(){
        EncoderDrive(DRIVE_SPEED, -13,13,13,-13,4.0); // Strafes out
        EncoderDrive(DRIVE_SPEED, 25.5,25.5,25.5,25.5,4.0); // Drive forward 24 inches
        EncoderDrive(DRIVE_SPEED,45,-45,45,-45,4.0); // rotate 180 degrees

    }






}   // end class
