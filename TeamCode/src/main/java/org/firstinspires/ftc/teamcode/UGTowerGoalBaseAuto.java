package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Tower Goal Auto", group="Thunderbots")
public class UGTowerGoalBaseAuto extends LinearOpMode {
    UGHardwarePushbot robot = new UGHardwarePushbot();
    private ElapsedTime runtime = new ElapsedTime();
    public static final double DRIVE_SPEED = 0.6;
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        //initSkystoneCamera();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Init done");    //
        telemetry.update();

        robot.leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        /*telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive1.getCurrentPosition(),
                robot.rightDrive1.getCurrentPosition());
        telemetry.update();*/

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        gettotargetline();
        strafelefttopowergoal();
        shootRingsIntoTowerGoal();
        moveforwardandpark();

            // Step through each leg of the path

    }

    public void gettotargetline() {
        //moveforwardtolaunchline


        robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

        encoderDrive(DRIVE_SPEED, 24, 24, 5.0);

    }

    public void strafelefttopowergoal() {
        //robot gets into position to shoot the power goals//


        robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);

        encoderDrive(DRIVE_SPEED, 24, 24, 5.0);


    }
    public void shootRingsIntoTowerGoal() {
        double shootPowerMultiplier = 0.5;

        robot.shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.shooterMotor.setPower(shootPowerMultiplier);

        for (int a = 1; a<=3;a++) {

            collectorUpDown(true); //lift collector
            sleep(1000);
            // each call of the trigger function moves trigger either in our out
            //So call it twice to move trigger in and then out
            trigger();
            trigger();
            sleep(1000);
            collectorUpDown(false); //bring collector down
            /*robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

            encoderDrive(DRIVE_SPEED, 24, 24, 0.2);*/
        }
        robot.shooterMotor.setPower(0);
    }


    public void moveforwardandpark() {

        sleep(500);
        //we'll enter the code for the shooter to shoot before this (once it is built)//

        double powerMultiplier = 0.5;

        robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

        encoderDrive(DRIVE_SPEED, 24, 24, 0.3);


    }
    public void trigger(){
        double triggerPosition = robot.triggerServo.getPosition();

        double MAX_POS = this.robot.triggerServo.MAX_POSITION;
        double MIN_POS = this.robot.triggerServo.MIN_POSITION;


        telemetry.addData("Trigger Activated; Current Position: ", triggerPosition);
        if (triggerPosition == MAX_POS ) {
            //triggerPosition += TRIGGERINCREMENT;
            triggerPosition =MIN_POS;
        }
        else if (triggerPosition==MIN_POS) {
            //triggerPosition -= TRIGGERINCREMENT;
            triggerPosition= MAX_POS;
        }
        else if (triggerPosition < (MIN_POS + (MAX_POS-MIN_POS)/2)){ //closer to min
            triggerPosition= MAX_POS;
        }
        else //closer to max
            triggerPosition= MIN_POS;
        robot.triggerServo.setPosition(triggerPosition);
        telemetry.addData("Trigger Activated; New Position: ", triggerPosition);
    }


    public void collectorUpDown(boolean liftCollector) {


        double collectorPosition = robot.collectorServo.getPosition();

        double MAX_POS = this.robot.collectorServo.MAX_POSITION;
        double MIN_POS = this.robot.collectorServo.MIN_POSITION;

        telemetry.addData("Collector position  ", collectorPosition);
        if (liftCollector ) {
            robot.collectorServo.setPosition(MAX_POS);
            telemetry.addData("Collector Lifted ", collectorPosition);


        } else  {
            robot.collectorServo.setPosition(MIN_POS);
            telemetry.addData("Collector Let Go ", collectorPosition);

        }

    }
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget1;
        int newRightTarget1;
        int newLeftTarget2;
        int newRightTarget2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget1 = robot.leftDrive1.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget1 = robot.rightDrive1.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftTarget2 = robot.leftDrive2.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget2 = robot.rightDrive2.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            robot.leftDrive1.setTargetPosition(newLeftTarget1);
            robot.rightDrive1.setTargetPosition(newRightTarget1);
            robot.leftDrive2.setTargetPosition(newLeftTarget2);
            robot.rightDrive2.setTargetPosition(newRightTarget2);

            // Turn On RUN_TO_POSITION
            robot.leftDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive1.setPower(Math.abs(speed));
            robot.rightDrive1.setPower(Math.abs(speed));
            robot.leftDrive2.setPower(Math.abs(speed));
            robot.rightDrive2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive1.isBusy() && robot.rightDrive1.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget1, newRightTarget1);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive1.getCurrentPosition(),
                        robot.rightDrive1.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive1.setPower(0);
            robot.rightDrive1.setPower(0);
            robot.leftDrive2.setPower(0);
            robot.rightDrive2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }

}