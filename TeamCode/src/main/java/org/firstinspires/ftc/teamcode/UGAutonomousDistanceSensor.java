package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bak.ThunderbotsSquareAutonomous;

@Autonomous(name="Distance Sensor Auto", group="Thunderbots")
public class UGAutonomousDistanceSensor extends UGTowerGoalBaseAuto {
    UGHardwarePushbot robot = new UGHardwarePushbot();
    private DistanceSensor distanceSensor = null;

    private ElapsedTime runtime = new ElapsedTime();
    double powerMultiplier = 0.4;
    double shooterPowerMultiplier = 1.0;
    double wobbledownpowerMultiplier = 0.6;
    double wobbleuppowerMultiplier = 0.6;
    double TRIGGERINCREMENT = 0.5;
    double strafePowerMultiplier = 0.5;



    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        //initSkystoneCamera();
        // Send telemetry message to signify robot waiting;
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        telemetry.addData("Status", "Init done");    //
        telemetry.update();

        robot.leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wobbleArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.wobbleArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        senseRings();



    }
public void senseRings() {

    robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
    robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
    robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
    robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
    encoderDrive(powerMultiplier, 24, 24, 1.0);

    double distanceServoPosition = this.robot.distanceServo.MAX_POSITION - 1.0;
    robot.distanceServo.setPosition(distanceServoPosition);

    telemetry.addData("deviceName",distanceSensor.getDeviceName() );
    telemetry.addData("range", String.format("%.01f mm", distanceSensor.getDistance(DistanceUnit.MM)));
    telemetry.update();

    if ((distanceSensor.getDistance(DistanceUnit.MM) >= 180 && (distanceSensor.getDistance(DistanceUnit.MM) <=200 ))) {
        {
            //MOVE FORWARD TO A SHOOTING POSITION


            robot.shooterMotor.setPower(-shooterPowerMultiplier);

            robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
            encoderDrive(powerMultiplier, 24, 24, 0.5);

            robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
            encoderDrive(powerMultiplier, 24, 24, 1.0);

            robot.rightDrive2.setPower(0);
            robot.rightDrive1.setPower(0);
            robot.leftDrive1.setPower(0);
            robot.leftDrive2.setPower(0);


            //shoot 3 pre-loaded rings
            double triggerPosition = robot.triggerServo.getPosition();
            double MAX_POS = this.robot.triggerServo.MAX_POSITION;
            double MIN_POS = this.robot.triggerServo.MIN_POSITION;

            for (int a = 1; a <= 10; a++) {
                sleep (500);
                triggerPosition = robot.triggerServo.getPosition();

                if (triggerPosition == MAX_POS ) {
                    //triggerPosition += TRIGGERINCREMENT;
                    triggerPosition =MIN_POS;
                    telemetry.addData("Trigger Min", triggerPosition);
                    telemetry.update();
                }
                else if (triggerPosition==MIN_POS) {
                    //triggerPosition -= TRIGGERINCREMENT;
                    triggerPosition= MAX_POS;
                    telemetry.addData("Trigger Max", triggerPosition);
                    telemetry.update();
                }
                else if (triggerPosition < (MIN_POS + (MAX_POS-MIN_POS)/2)){ //closer to min
                    triggerPosition= MAX_POS;
                    telemetry.addData("Trigger Max", triggerPosition);
                    telemetry.update();
                }
                else {//closer to max
                    triggerPosition = MIN_POS;
                    telemetry.addData("Trigger Min", triggerPosition);
                    telemetry.update();
                }
                robot.triggerServo.setPosition(triggerPosition);
                telemetry.addData("A", a);
                telemetry.update();
            }

            robot.shooterMotor.setPower(0);

            //robot moves onto the live
            robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
            //MOVE FORWARD TO A DROP POSITION
            encoderDrive(powerMultiplier, 24, 24, 0.4);
            //Wobble arm moves down
            robot.wobbleArmMotor.setPower(wobbledownpowerMultiplier);
            wobbleDrive(wobbledownpowerMultiplier, 5,  1.0);
            robot.wobbleArmMotor.setPower(0);
            sleep (1000);
            //Open Claw
            telemetry.addData("Status", "wobbleclaw is here");
            telemetry.update();

            double wobbleClawPosition = this.robot.wobbleClawServo.MAX_POSITION - 1.0;
            robot.wobbleClawServo.setPosition(wobbleClawPosition);

            robot.rightDrive2.setPower(0);
            robot.rightDrive1.setPower(0);
            robot.leftDrive1.setPower(0);
            robot.leftDrive2.setPower(0);
            //Wobble Arm Up
            robot.wobbleArmMotor.setPower(-wobbledownpowerMultiplier);
            wobbleDrive(-wobbledownpowerMultiplier, 5,  0.9);
            robot.wobbleArmMotor.setPower(0);




        }


    }

   else if ((distanceSensor.getDistance(DistanceUnit.MM) >= 160 && (distanceSensor.getDistance(DistanceUnit.MM) < 180 ))) {
        robot.shooterMotor.setPower(-shooterPowerMultiplier);

        //MOVE FORRWARD TO A SHOOTING POSITION
        robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderDrive(powerMultiplier, 24, 24, 1.3); //used to be 1.5

        robot.rightDrive2.setPower(0);
        robot.rightDrive1.setPower(0);
        robot.leftDrive1.setPower(0);
        robot.leftDrive2.setPower(0);
        //SHOOT
        double triggerPosition = robot.triggerServo.getPosition();
        double MAX_POS = this.robot.triggerServo.MAX_POSITION;
        double MIN_POS = this.robot.triggerServo.MIN_POSITION;

        for (int a = 1; a <= 10; a++) {
            sleep (500);
            triggerPosition = robot.triggerServo.getPosition();

            if (triggerPosition == MAX_POS ) {
                //triggerPosition += TRIGGERINCREMENT;
                triggerPosition =MIN_POS;
                telemetry.addData("Trigger Min", triggerPosition);
                telemetry.update();
            }
            else if (triggerPosition==MIN_POS) {
                //triggerPosition -= TRIGGERINCREMENT;
                triggerPosition= MAX_POS;
                telemetry.addData("Trigger Max", triggerPosition);
                telemetry.update();
            }
            else if (triggerPosition < (MIN_POS + (MAX_POS-MIN_POS)/2)){ //closer to min
                triggerPosition= MAX_POS;
                telemetry.addData("Trigger Max", triggerPosition);
                telemetry.update();
            }
            else {//closer to max
                triggerPosition = MIN_POS;
                telemetry.addData("Trigger Min", triggerPosition);
                telemetry.update();
            }
            robot.triggerServo.setPosition(triggerPosition);
            telemetry.addData("A", a);
            telemetry.update();
        }

        robot.shooterMotor.setPower(0);


        //robot moves onto the live
        robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        //MOVE FORRWARD TO A DROP POSITION
        encoderDrive(strafePowerMultiplier, 24, 24, 0.5);

        robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        //MOVE FORRWARD TO A DROP POSITION
        encoderDrive(powerMultiplier, 24, 24, 1.0);
        //Wobble arm moves down
        robot.wobbleArmMotor.setPower(wobbledownpowerMultiplier);
        wobbleDrive(wobbledownpowerMultiplier, 5,  1.0);
        robot.wobbleArmMotor.setPower(0);
        sleep(1000);
        //Open Claw
        telemetry.addData("Status", "wobbleclaw is here");
        telemetry.update();

        double wobbleClawPosition = this.robot.wobbleClawServo.MAX_POSITION - 1.0;
        robot.wobbleClawServo.setPosition(wobbleClawPosition);

        robot.rightDrive2.setPower(0);
        robot.rightDrive1.setPower(0);
        robot.leftDrive1.setPower(0);
        robot.leftDrive2.setPower(0);
        //Wobble Arm Up
        robot.wobbleArmMotor.setPower(wobbledownpowerMultiplier);
        wobbleDrive(-wobbledownpowerMultiplier, 5,  0.9);
        robot.wobbleArmMotor.setPower(0);


        robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        //MOVE FORRWARD TO A DROP POSITION
        encoderDrive(powerMultiplier, 24, 24, 1.0);

        robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        //MOVE FORRWARD TO A DROP POSITION
        encoderDrive(powerMultiplier, 24, 24, 0.2);

    }
    else if ((distanceSensor.getDistance(DistanceUnit.MM) >= 100 && (distanceSensor.getDistance(DistanceUnit.MM) <=140))) {
        robot.shooterMotor.setPower(-shooterPowerMultiplier);

        //MOVE FORRWARD TO A SHOOTING POSITION
        robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderDrive(powerMultiplier, 24, 24, 0.5);

        robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderDrive(powerMultiplier, 24, 24, 1.0);

        robot.rightDrive2.setPower(0);
        robot.rightDrive1.setPower(0);
        robot.leftDrive1.setPower(0);
        robot.leftDrive2.setPower(0);
        //SHOOT
        double triggerPosition = robot.triggerServo.getPosition();
        double MAX_POS = this.robot.triggerServo.MAX_POSITION;
        double MIN_POS = this.robot.triggerServo.MIN_POSITION;

        for (int a = 1; a <= 15; a++) {
            sleep (500);
            triggerPosition = robot.triggerServo.getPosition();

            if (triggerPosition == MAX_POS ) {
                //triggerPosition += TRIGGERINCREMENT;
                triggerPosition =MIN_POS;
                telemetry.addData("Trigger Min", triggerPosition);
                telemetry.update();
            }
            else if (triggerPosition==MIN_POS) {
                //triggerPosition -= TRIGGERINCREMENT;
                triggerPosition= MAX_POS;
                telemetry.addData("Trigger Max", triggerPosition);
                telemetry.update();
            }
            else if (triggerPosition < (MIN_POS + (MAX_POS-MIN_POS)/2)){ //closer to min
                triggerPosition= MAX_POS;
                telemetry.addData("Trigger Max", triggerPosition);
                telemetry.update();
            }
            else {//closer to max
                triggerPosition = MIN_POS;
                telemetry.addData("Trigger Min", triggerPosition);
                telemetry.update();
            }
            robot.triggerServo.setPosition(triggerPosition);
            telemetry.addData("A", a);
            telemetry.update();
        }

        robot.shooterMotor.setPower(0);

        sleep(1000);
        //robot moves onto the live
        robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        //MOVE FORRWARD TO A DROP POSITION
        encoderDrive(powerMultiplier, 24, 24, 2.0);
        //Wobble arm moves down
        robot.wobbleArmMotor.setPower(wobbledownpowerMultiplier);
        wobbleDrive(wobbledownpowerMultiplier, 5,  1.0);
        robot.wobbleArmMotor.setPower(0);
        sleep(1000);
        //Open Claw
        telemetry.addData("Status", "wobbleclaw is here");
        telemetry.update();

        double wobbleClawPosition = this.robot.wobbleClawServo.MAX_POSITION - 1.0;
        robot.wobbleClawServo.setPosition(wobbleClawPosition);

        robot.rightDrive2.setPower(0);
        robot.rightDrive1.setPower(0);
        robot.leftDrive1.setPower(0);
        robot.leftDrive2.setPower(0);
        //Wobble Arm Up
        robot.wobbleArmMotor.setPower(wobbledownpowerMultiplier);
        wobbleDrive(-wobbledownpowerMultiplier, 5,  0.9);
        robot.wobbleArmMotor.setPower(0);


        robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        //MOVE FORRWARD TO A DROP POSITION
        encoderDrive(powerMultiplier, 24, 24, 1.2);

    }

    else   {

        robot.shooterMotor.setPower(-shooterPowerMultiplier);

        //MOVE FORRWARD TO A SHOOTING POSITION
        robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderDrive(powerMultiplier, 24, 24, 1.3); //used to be 1.5

        robot.rightDrive2.setPower(0);
        robot.rightDrive1.setPower(0);
        robot.leftDrive1.setPower(0);
        robot.leftDrive2.setPower(0);
        //SHOOT
        double triggerPosition = robot.triggerServo.getPosition();
        double MAX_POS = this.robot.triggerServo.MAX_POSITION;
        double MIN_POS = this.robot.triggerServo.MIN_POSITION;

        for (int a = 1; a <= 10; a++) {
            sleep (500);
            triggerPosition = robot.triggerServo.getPosition();

            if (triggerPosition == MAX_POS ) {
                //triggerPosition += TRIGGERINCREMENT;
                triggerPosition =MIN_POS;
                telemetry.addData("Trigger Min", triggerPosition);
                telemetry.update();
            }
            else if (triggerPosition==MIN_POS) {
                //triggerPosition -= TRIGGERINCREMENT;
                triggerPosition= MAX_POS;
                telemetry.addData("Trigger Max", triggerPosition);
                telemetry.update();
            }
            else if (triggerPosition < (MIN_POS + (MAX_POS-MIN_POS)/2)){ //closer to min
                triggerPosition= MAX_POS;
                telemetry.addData("Trigger Max", triggerPosition);
                telemetry.update();
            }
            else {//closer to max
                triggerPosition = MIN_POS;
                telemetry.addData("Trigger Min", triggerPosition);
                telemetry.update();
            }
            robot.triggerServo.setPosition(triggerPosition);
            telemetry.addData("A", a);
            telemetry.update();
        }

        robot.shooterMotor.setPower(0);


        //robot moves onto the live
        robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        //MOVE FORRWARD TO A DROP POSITION
        encoderDrive(strafePowerMultiplier, 24, 24, 0.5);
    }

}



    public void wobbleDrive (double speed,
                             double Inches,
                             double timeoutS) {
        int newWobbleTarget;

        if (opModeIsActive()) {
            newWobbleTarget = robot.wobbleArmMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            robot.wobbleArmMotor.setTargetPosition(newWobbleTarget);

            robot.wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.wobbleArmMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.wobbleArmMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path3", "Running to %7d :%7d", newWobbleTarget,
                        robot.wobbleArmMotor.getCurrentPosition());
                telemetry.update();
            }
            robot.wobbleArmMotor.setPower(0);

            robot.wobbleArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }

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


        }
    }

}





