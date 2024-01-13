package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "CRAuto Blue P1")
public class CRAuto extends LinearOpMode {

//change for code
    //Tfod Processor set up
    TfodProcessor tfod = TfodProcessor.easyCreateWithDefaults();

    //April Tag Processor Class
    AprilTagProcessor my_AprilTagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(false) //doesn't draw XYZ axes
            .setDrawCubeProjection(false)//doesn't draw a cube around the tag
            .setDrawTagID(true)//draws ID on the tag
            .setDrawTagOutline(true)//draws tag outline around the black border
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
            .build();

    //Sets up a Vision Portal class
    VisionPortal visionPortal = new VisionPortal.Builder()
            .addProcessor(my_AprilTagProcessor)
            .addProcessor(tfod)
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .setCameraResolution(new Size(640, 480))
            .enableLiveView(true)//streams to the driver hub?
            .build();

    //To analyze the distance/orientation of the robot in comparison to the April Tag TELEMETRY
   /* private int telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = my_AprilTagProcessor.getFreshDetections(); //April Tag Detection is a class
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections)
        {
            if (detection.metadata != null)
            {
                if(!currentDetections.contains(my_AprilTagProcessor.getDetections())) //checks that the current detection isn't already in the list
                {
                    AprilTagDetection.add(my_AprilTagProcessor.getDetections());
                }

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name)); //gets the ID code, tag name ex: 583, Nemo

            }
            else
            {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id)); //says detection id is unkown
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag() */

    private int telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = my_AprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        //int my_AprilTagID = 100;
        AprilTagDetection my_AprilTagID = currentDetections.get(0); //stores id of first detection

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections)
        {
            if (detection.metadata != null)
            {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

            } else
            {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        return my_AprilTagID.id; //returns first detection id

    }   // end method telemetryAprilTag()




    @Override

    public void runOpMode() throws InterruptedException {

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        //setting up a variable for the timer
        ElapsedTime runtime = new ElapsedTime();

        //Wait for driver to press button and timer resets
        waitForStart();
        resetRuntime();


        while(opModeIsActive() && runtime.seconds() < 30)
        {


           /* while(my_AprilTagProcessor.getDetections().size() == 0)
            {
                //Run the Encoder code and tfod code to place the pixel on the spike mark then turn to face the april tag
                //then it will break out of this while loop
            }
            //Only needed while in the process of detecting an april tag
           // if(aprilTagProcessor.getDetections().size() > 0) //ensures at least one april tag is in view
           */

            telemetryAprilTag(); //gets and saves the reference tag as an int TagID and prints the ID number



           /* if(TagID == 1)
                {
                    //Drive over to the leftmost ID for blue

                }
                else if(TagID == 2)
                {
                    //Drive over to the center most ID for blue
                }
                else if(TagID == 3)
                {
                    //drive over to the right ID for blue
                } */









        }//end of Active Op Mode


    }//end class
}//end main
