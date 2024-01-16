package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
//
@TeleOp
public class curiosityRovers extends LinearOpMode {
    // Initialize Drive motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    // Initialize Intake Motor
    private DcMotor intakeMotor;

    private DcMotor outtakeMotor = null;


    // Initialize lift motors. NOTE double ticks = 1425.1 is one revolution. 25 revolutions to reach the top: 35627.5 ticks. Ticks For better motor 145.6. 3640
    private DcMotor Lift = null;

    // Initialize Linear slide motor and dropper motor
    private Servo Dropper = null;
    private double servody = 0.01;
    private double servoPosition = 0.58;
    double topPosition = 13308.07;


    public void runOuttakeMotor(double power) {
        outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotor.setPower(power);
    }
    public void servoDefault() {
        servoPosition = 0.58;
        servoPosition = Range.clip(servoPosition, 0, 0.58);
        Dropper.setPosition(servoPosition);
    }
    public void servoHold() {
        servoPosition = 0.4;
        servoPosition = Range.clip(servoPosition, 0, 0.58);
        Dropper.setPosition(servoPosition);
    }
    public void servoDrop() {
        servoPosition = 0;
        servoPosition = Range.clip(servoPosition, 0, 0.58);
        Dropper.setPosition(servoPosition);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // initialize telemetry
        telemetry.addData("status","initialized");
        telemetry.update();

        //add motor config map
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        //set motor direction
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);


        // Get the Encoder Moder from Control Hub
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setDirection(DcMotorSimple.Direction.FORWARD);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Get Servo from Control hub and initialize position.
        Servo leftClaw = hardwareMap.get(Servo.class, "leftClaw");

        // Get Intake Motor from Control Hub - Set to reverse direction
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        double intakePower = 0.5;

        // get outtake Motor and Dropper motor
        outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");
        Dropper = hardwareMap.get(Servo.class, "Dropper");

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        //waitForStart
        waitForStart();

        while(opModeIsActive()){
            // DRIVE CODE

            // Get gamepad input variables for movement
            double speed = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x * 1.1;
            double pivot = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(speed) +
                    Math.abs(strafe) + Math.abs(pivot), 1);

            double leftFrontPower = (speed + strafe + pivot)/denominator;
            double rightFrontPower = (speed - strafe - pivot)/denominator;
            double leftRearPower = (speed - strafe + pivot)/denominator;
            double rightRearPower = (speed + strafe - pivot)/denominator;

            // Powers motors in the set directions based off the gamepad
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftRear.setPower(leftRearPower);
            rightRear.setPower(rightRearPower);

            // INTAKE MOTOR CODE
            if (gamepad1.left_bumper) {
                intakePower = 0.75;
                intakeMotor.setPower(intakePower);
            } else if (gamepad1.a){
                intakePower = 0;
                intakeMotor.setPower(intakePower);
            } else if (gamepad1.right_bumper) {
                intakePower = -0.75;
                intakeMotor.setPower(intakePower);
            }

            //LIFT CODE

            // Lift encoders
            if (gamepad2.y) {
                Lift.setTargetPosition((int) (topPosition));
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(1);
            } else if (gamepad2.a){
                Lift.setTargetPosition(5); // Target position should be set slightly higher than zero so the motor gears do not jam
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(1);
            }
            // Servo positions
            if (gamepad2.x) {
                leftClaw.setPosition(0);
            } else if (gamepad2.b) {
                leftClaw.setPosition(.52);
            }
            // Linear slide code

            if (gamepad2.dpad_up) {
                runOuttakeMotor(0.8);
            } else if (gamepad2.dpad_down) {
                runOuttakeMotor(-0.8);
            } else {
                runOuttakeMotor(0);
            }

            if (gamepad2.left_bumper) {
                servoHold();
            } else if (gamepad2.right_bumper) {
                servoDrop();
            }else
                servoDefault();


            //Update Telemetry Lift motor and Servo Positions.
            telemetry.addData("Lift Position: ", Lift.getCurrentPosition());
            telemetry.addData("Servo Position: ", leftClaw.getPosition());

            //Update IntakePower
            telemetry.addData("IntakePower set: ", intakePower);

            telemetry.update();




        }


    }

}

