package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="A: UGOpModeQT", group="Thunderbots")
public class UGOpMode_DecQT extends LinearOpMode {
    UGHardwarePushbot robot = new UGHardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    double leftForwardPower;
    double rightForwardPower;
    double leftBackwardPower;
    double rightBackwardPower;
    double intakePower;
    double intakeHighPower;
    double shooterPower;

    double MAX_POS = 3.0;     // Maximum rotational position
    double MIN_POS = 0.0;     // Minimum rotational position

    double wobbleClawPositon = 0;
    double collectorPosition = 0;
    double triggerPosition = 0;

    boolean startIntake = false;
    boolean stopIntake = true;
    boolean startIntakeHigh = false;
    boolean stopIntakeHigh = false;
    boolean startWheel = false;
    boolean stopWheel = true;
    boolean collectorUp = false;

    boolean triggerused = false;
    boolean triggerback = true;


    double powerMultiplier = 1.0; // 1.0
    double CLAWINCREMENT = 0.4; //may have to adjust, check before finalizing
    double COLLECTORINCREMENT = 1.0;
    double TRIGGERINCREMENT = 1.0;
    double ParkpowerMultiplier = .9;
    double MAX_POWER = 1.0;    // 1.00
    double POWER_INCREMENT = 0.2;
    double powerMultiplierArm = -0.8;

    // private Servo grabber = null;
    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Init Done");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Each of these functions checks for specific buttons on gamepads and does the corresponding action
            driveMacChasis();
            startStopIntake();
            collectorUpDown();
            shootRing();
            wobbleGoal();
            telemetry.update();
        }

    }

    public void driveMacChasis() {
        // POV Mode uses left stick to go forward, backward, and turn
        // - This uses basic math to combine motions and is easier to drive straight.

        double driveForward = gamepad1.left_stick_y;
        double driveBackward = gamepad1.left_stick_y;
        double turnRight = gamepad1.right_stick_x;
        double turnLeft = gamepad1.right_stick_x;
        double strafeRight = gamepad1.left_stick_x;
        double strafeLeft = gamepad1.left_stick_x;
        //double powerMultiplier = 0.5;

        boolean driveStop = false;


        if ((gamepad1.left_stick_y == 0) && (gamepad1.right_stick_y == 0) && (gamepad1.left_stick_x == 0) && (gamepad1.right_stick_x == 0))
            driveStop = true;

        //Mecanum wheels work well with full power
        if (turnRight < 0) {
            telemetry.addData("Status", "Moving right");
            telemetry.update();

            robot.leftDrive1.setPower(powerMultiplier);
            robot.rightDrive1.setPower(-powerMultiplier);
            robot.leftDrive2.setPower(powerMultiplier);
            robot.rightDrive2.setPower(-powerMultiplier);

        } else if (turnLeft > 0) {

            telemetry.addData("Status", "Moving left");
            telemetry.update();

            robot.leftDrive1.setPower(-powerMultiplier);
            robot.rightDrive1.setPower(powerMultiplier);
            robot.leftDrive2.setPower(-powerMultiplier);
            robot.rightDrive2.setPower(powerMultiplier);

        } else if (driveForward > 0) {

            telemetry.addData("Status", "Moving forward");
            telemetry.update();

            robot.leftDrive1.setPower(powerMultiplier);
            robot.rightDrive1.setPower(powerMultiplier);
            robot.leftDrive2.setPower(powerMultiplier);
            robot.rightDrive2.setPower(powerMultiplier);
        } else if (driveBackward < 0) {

            telemetry.addData("Status", "Moving backward");
            telemetry.update();

            robot.leftDrive1.setPower(-powerMultiplier);
            robot.rightDrive1.setPower(-powerMultiplier);
            robot.leftDrive2.setPower(-powerMultiplier);
            robot.rightDrive2.setPower(-powerMultiplier);
        } else if (driveStop) {
            //telemetry.addData("Status", "Stopping");
            //telemetry.update();

            robot.leftDrive1.setPower(0);
            robot.rightDrive1.setPower(0);
            robot.leftDrive2.setPower(0);
            robot.rightDrive2.setPower(0);
        } else if (strafeRight < 0) {
            telemetry.addData("Status", "Moving Right");
            telemetry.update();

            robot.leftDrive1.setPower(powerMultiplier);
            robot.rightDrive1.setPower(-powerMultiplier);
            robot.leftDrive2.setPower(-powerMultiplier);
            robot.rightDrive2.setPower(powerMultiplier);

        } else if (strafeLeft > 0) {
            telemetry.addData("Status", "Moving Left");
            telemetry.update();

            robot.leftDrive1.setPower(-powerMultiplier);
            robot.rightDrive1.setPower(powerMultiplier);
            robot.leftDrive2.setPower(powerMultiplier);
            robot.rightDrive2.setPower(-powerMultiplier);
        }

        leftForwardPower = this.robot.leftDrive1.getPower();
        rightForwardPower = this.robot.rightDrive1.getPower();
        leftBackwardPower = this.robot.leftDrive1.getPower();
        rightBackwardPower = this.robot.rightDrive1.getPower();
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Inputs received", "Drive Forward: " + driveForward + " Drive Backward: " + driveBackward + " Drive Right: " + turnRight + " Drive Left: " + turnLeft);
        telemetry.addData("Motors Forward", "left (%.2f), right (%.2f)", leftForwardPower, rightForwardPower);
        telemetry.addData("Motors Backward", "left (%.2f), right (%.2f)", leftBackwardPower, rightBackwardPower);


    }

    public void startStopIntake() {

        boolean toggleIntakePressed = gamepad1.y;

        if (toggleIntakePressed ) {

            if (startIntake && startWheel && startIntakeHigh) { //Intake is already running. So Stop it.
                stopIntake = true;
                startIntake = false;
                stopWheel = true;
                startWheel = false;
                stopIntakeHigh = true;
                startIntakeHigh = false;
            } else { //the Intake is not running. So Start it.
                startIntake = true;
                stopIntake = false;
                startWheel = true;
                stopWheel = false;
                startIntakeHigh = true;
                stopIntakeHigh = false;
            }


            intakePower = this.robot.intakeMotorLow.getPower();
            shooterPower = this.robot.shooterMotor.getPower();
            intakeHighPower = this.robot.intakeMotorHigh.getPower();
            if (stopIntake && stopWheel && stopIntakeHigh) { //checking the power of the motors
                    robot.intakeMotorLow.setPower(0);//stop the motors
                    robot.shooterMotor.setPower(0);
                    robot.intakeMotorHigh.setPower(0);
                    telemetry.addData("Status", "Stopping intake..");
            }
            if (startIntake && startWheel && startIntakeHigh) {
                    robot.intakeMotorLow.setPower(powerMultiplier);
                    robot.shooterMotor.setPower(powerMultiplier);
                    robot.intakeMotorHigh.setPower(-powerMultiplier);
                telemetry.addData("Status", "Starting intake..");
            }

            telemetry.update();

        }
    }

    public void collectorUpDown() {
        boolean liftCollector = gamepad2.y;
        boolean letGoCollector = gamepad2.a;

        MAX_POS = this.robot.collectorServo.MAX_POSITION;
        MIN_POS = this.robot.collectorServo.MIN_POSITION;
        if (liftCollector) {
            telemetry.addData("Collector Lifted", collectorPosition);
            if (wobbleClawPositon <= MAX_POS) {
                wobbleClawPositon += COLLECTORINCREMENT;
            }
            robot.wobbleClawServo.setPosition(collectorPosition);
        } else if (letGoCollector) {
            telemetry.addData("Collector Down", collectorPosition);
            if (wobbleClawPositon >= MIN_POS) {
                wobbleClawPositon -= COLLECTORINCREMENT;
                robot.wobbleClawServo.setPosition(collectorPosition);

            }


        }
    }
    public void shootRing() {
        boolean activateTrigger = gamepad2.x;

        MAX_POS = this.robot.triggerServo.MAX_POSITION;
        MIN_POS = this.robot.triggerServo.MIN_POSITION;
        if (activateTrigger) {
            telemetry.addData("Trigger Activated", triggerPosition);
            if (triggerPosition <= MAX_POS) {
                triggerPosition += TRIGGERINCREMENT;
                triggerPosition -= TRIGGERINCREMENT;
            }
        }
    }



            public void wobbleGoal () {
                boolean wobbleClawOpen = gamepad2.dpad_left;
                boolean wobbleClawClose = gamepad2.dpad_right;
                float wobbleArmUp = gamepad2.left_trigger;
                float wobbleArmDown = gamepad2.right_trigger;

                MAX_POS = this.robot.wobbleClawServo.MAX_POSITION;
                MIN_POS = this.robot.wobbleClawServo.MIN_POSITION;

                if (wobbleArmUp > 0.5) {
                    robot.wobbleArmMotor.setPower(powerMultiplier);
                }
                else if (wobbleArmUp == 0) {
                    robot.wobbleArmMotor.setPower(0);
                }

                if (wobbleArmDown > 0.5) {
                    robot.wobbleArmMotor.setPower(-powerMultiplier);
                }
                else if (wobbleArmDown == 0) {
                    robot.wobbleArmMotor.setPower(0);
                }



                 if (wobbleClawOpen) {
                    telemetry.addData("Claw open", wobbleClawPositon);
                    if (wobbleClawPositon <= MAX_POS) {
                        wobbleClawPositon += CLAWINCREMENT;
                    }
                    robot.wobbleClawServo.setPosition(wobbleClawPositon);
                } else if (wobbleClawClose) {
                    telemetry.addData("Claw close", wobbleClawPositon);
                    if (wobbleClawPositon >= MIN_POS) {
                        wobbleClawPositon -= CLAWINCREMENT;
                        robot.wobbleClawServo.setPosition(wobbleClawPositon);

                    }

                }
            }


}