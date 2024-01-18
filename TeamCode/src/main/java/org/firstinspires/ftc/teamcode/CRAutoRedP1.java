package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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


@Autonomous(name = "CRAutoRedP1", group = "Concept")
public class CRAutoRedP1 extends LinearOpMode {

    //April Tag Variables
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag; //variable stores instance of April Tag Processor
    private VisionPortal visionPortal; //Variable stores instance of the vision portal

    int myTagID; //Will store the int type for the id number


    //TFOD Variables
    private static final String TFOD_MODEL_ASSET = "RedMarker.tflite";
    private static final String[] LABELS = {"Red Marker",};

    private TfodProcessor tfod; //The variable to store our instance of the TensorFlow Object Detection processor.

    private VisionPortal visionPortal2; //The variable to store our instance of the vision portal.

    float confidMin = 0.80f; //sets min confidence level
    int xSizeMin = 100; //sets min x width
    int xSizeMax = 175; //sets max x width
    int ySizeMin = 90; //sets min y height
    int ySizeMax = 160; //sets max y height
    int yMin = 280; //sets y coord min
    int yMax = 400; //sets y coord max
    int xMinSMC = 1; //sets min x coord for spike mark right
    int xMaxSMC = 300; //sets max x coord for spike mark right
    int xMinSMR = 301; //sets min x coord for spike mark center
    int xMaxSMR = 640; //sets max x coord for spike mark center
    int RightMark = 0;
    int CenterMark = 1;
    int LeftMark = 2;
    int SpikeMarkLocation = LeftMark;



    @Override
    public void runOpMode()
    {

        //TFOD op mode part
        initTfod();


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive())
        {
            int SpikeMarkLocationFinal = scanTFOD(); //for loop method where Spike Mark Location = telemetryTFOD();


            if(SpikeMarkLocationFinal == RightMark)
            {
                telemetry.addLine("Path for Spike Mark Right is starting");

            }
            else if (SpikeMarkLocationFinal == CenterMark)
            {
                telemetry.addLine("Path for Spike Mark Center is starting");
            }
            else
            {
                telemetry.addLine("Path for Spike Mark Left is starting");
            }
            telemetry.update();

            visionPortal2.close();



            //April Tag Portion of OP Mode
            initAprilTag();

            scanAprilTag();


            if(myTagID == 4)
            {
                telemetry.addLine("ALEX: The Path for Tag ID 4 will be started");
                //encoder code here for ID 1
            }
            else if(myTagID == 5)
            {
                telemetry.addLine("ALEX 2: The path for ID 5 will be started");
                //encoder code here for ID 2
            }
            else if(myTagID == 6)
            {
                telemetry.addLine("Alex 3: The path for ID 6 will be started");
                //encoder code here for ID 3
            }
            else
            {
                telemetry.addLine("The April Tag did not match 4,5,or 6");
                //encoder code here for default
            }
            telemetry.update();

            // Share the CPU.
            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()


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
                break;
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




}   // end class
