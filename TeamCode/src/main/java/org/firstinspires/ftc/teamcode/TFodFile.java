package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "TFodFile")
public class TFodFile extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "BlueMarker.tflite";

    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.

    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";

    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Blue Marker",
    };

    private TfodProcessor tfod; //The variable to store our instance of the TensorFlow Object Detection processor.

    private VisionPortal visionPortal; //The variable to store our instance of the vision portal.

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
    public void runOpMode() {

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

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



                //}   // end method telemetryTfod()

                // Push telemetry to the Driver Station.
                //telemetry.update();

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

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
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

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.

        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(confidMin);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

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
