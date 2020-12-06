package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class UGHardwarePushbot {

        /* Public OpMode members. */
        //Macenum has 4 wheels each with a motor
        public DcMotor leftDrive1   = null;
        public DcMotor  rightDrive1  = null;
        public DcMotor  leftDrive2   = null;
        public DcMotor  rightDrive2  = null;

        //Additional 3 motors apart from wheel motors
        public DcMotor intakeMotorLow = null;
        public DcMotor intakeMotorHigh = null;
        public DcMotor wobbleArmMotor = null;
        public DcMotor shooterMotor = null;

        // 3 servos used
        public Servo wobbleClawServo = null;
        public Servo collectorServo = null;
        public Servo triggerServo = null;

        //Distance sensor to be added


        /* local OpMode members. */
        HardwareMap hwMap           =  null;
        private ElapsedTime period  = new ElapsedTime();


        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize 4 Motors
            leftDrive1  = hwMap.get(DcMotor.class, "left_drive1");
            rightDrive1 = hwMap.get(DcMotor.class, "right_drive1");
            leftDrive2  = hwMap.get(DcMotor.class, "left_drive2");
            rightDrive2 = hwMap.get(DcMotor.class, "right_drive2");

            intakeMotorLow = hwMap.get(DcMotor.class, "intake_low");
            intakeMotorHigh = hwMap.get(DcMotor.class,"intake_high");
            wobbleArmMotor = hwMap.get(DcMotor.class, "wobble_arm");
            shooterMotor = hwMap.get(DcMotor.class,"shooter");

            wobbleClawServo = hwMap.get(Servo.class, "wobble_claw");
            collectorServo = hwMap.get(Servo.class, "collector");
            triggerServo = hwMap.get(Servo.class, "trigger");

            //Initializing all motors with FORWARD direction
            leftDrive1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            rightDrive1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
            leftDrive2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            rightDrive2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

            intakeMotorLow.setDirection(DcMotor.Direction.FORWARD);
            intakeMotorHigh.setDirection(DcMotor.Direction.FORWARD);
            wobbleArmMotor.setDirection(DcMotor.Direction.FORWARD);
            shooterMotor.setDirection(DcMotor.Direction.FORWARD);

            // Set all motors to zero power
            leftDrive1.setPower(0);
            rightDrive1.setPower(0);
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);
            intakeMotorLow.setPower(0);
            intakeMotorHigh.setPower(0);
            wobbleArmMotor.setPower(0);
            shooterMotor.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            leftDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotorLow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotorHigh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wobbleArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //Setting all Servos to FORWARD direction
            wobbleClawServo.setDirection(Servo.Direction.FORWARD);
            collectorServo.setDirection(Servo.Direction.FORWARD);
            triggerServo.setDirection(Servo.Direction.FORWARD);

        }
    }



