package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class UGHardwarePushbot {

        /* Public OpMode members. */
        //Macenum has 4 wheels each with a motor
        public DcMotor leftDrive1   = null;
        public DcMotor  rightDrive1  = null;
        public DcMotor  leftDrive2   = null;
        public DcMotor  rightDrive2  = null;
        public DcMotor intake = null;
        public DcMotor wobbleArm = null;
        public DcMotor shooter = null;

        public Servo wobbleClaw = null;
        public Servo collector = null;
        public Servo trigger = null;

        //Distance sensor to be added


        /* local OpMode members. */
        HardwareMap hwMap           =  null;
        private ElapsedTime period  = new ElapsedTime();

        /* Constructor */
        public UGHardwarePushbot(){

        }

        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize 4 Motors
            leftDrive1  = hwMap.get(DcMotor.class, "left_drive1");
            rightDrive1 = hwMap.get(DcMotor.class, "right_drive1");
            leftDrive2  = hwMap.get(DcMotor.class, "left_drive2");
            rightDrive2 = hwMap.get(DcMotor.class, "right_drive2");

            intake = hwMap.get(DcMotor.class, "intake");
            wobbleArm = hwMap.get(DcMotor.class, "wobble_arm");
            shooter = hwMap.get(DcMotor.class,"shooter");

            wobbleClaw = hwMap.get(Servo.class, "wobble_claw");
            collector = hwMap.get(Servo.class, "collector");
            trigger = hwMap.get(Servo.class, "trigger");



            leftDrive1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            rightDrive1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
            leftDrive2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            rightDrive2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

            intake.setDirection(DcMotor.Direction.FORWARD);
            wobbleArm.setDirection(DcMotor.Direction.FORWARD);
            shooter.setDirection(DcMotor.Direction.FORWARD);

            // Set all motors to zero power
            leftDrive1.setPower(0);
            rightDrive1.setPower(0);
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);

            intake.setPower(0);
            wobbleArm.setPower(0);
            shooter.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            leftDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            wobbleClaw.setDirection(Servo.Direction.FORWARD);
            collector.setDirection(Servo.Direction.FORWARD);
            trigger.setDirection(Servo.Direction.FORWARD);

        }
    }



