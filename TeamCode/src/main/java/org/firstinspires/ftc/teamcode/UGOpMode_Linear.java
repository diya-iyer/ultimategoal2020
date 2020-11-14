package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="A: UGOpMode", group="Thunderbots")
public class UGOpMode_Linear extends LinearOpMode {
    UGHardwarePushbot robot = new UGHardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    double leftForwardPower;
    double rightForwardPower;
    double leftBackwardPower;
    double rightBackwardPower;
    double intakePower;
    double shooterPower;

    double MAX_POS = 3.0;     // Maximum rotational position
    double MIN_POS = 0.0;     // Minimum rotational position

    double wobbleClawPositon = 0;
    double collectorPosition = 0;
    double triggerPosition = 0;

    boolean startInhaler = false;
    boolean stopInhaler = true;
    boolean startWheel = false;
    boolean stopWheel = true;
    boolean collectorUp = false;
    boolean collectordown = true;


    double powerMultiplier = 1.0; // 1.0
    double CLAWINCREMENT = 1.0; //may have to adjust, check before finalizing
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

        //double tgtPower = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry

            driveMacChasis();
            collectRing();
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

    public void collectRing() {

        boolean inhaleRing = gamepad1.y;
        boolean shootRing = gamepad1.y;

        if (inhaleRing && shootRing) {

            if (startInhaler && startWheel) { //inhaler is already running
                stopInhaler = true;
                startInhaler = false;
                stopWheel = true;
                startWheel = false;
            } else { //the inhaler is not running
                startInhaler = true;
                stopInhaler = false;
                startWheel = true;
                stopWheel = false;
            }

            while (true) {
                intakePower = this.robot.intake.getPower();
                shooterPower = this.robot.shooter.getPower();
                if (stopInhaler && stopWheel) { //checking the power of the motors
                    robot.intake.setPower(0);//stop the motors
                    robot.shooter.setPower(0);
                    break;
                }
                if (startInhaler && startWheel) { //we have to keep setting the power as long as startInhaler is true
                    robot.intake.setPower(powerMultiplier);
                    robot.shooter.setPower(powerMultiplier);
                }
                telemetry.addData("Status", "Inhaling Ring");
                telemetry.update();
            }
        }
    }

    public void shootRing() {
        boolean liftCollector = gamepad1.x;
        boolean activateTrigger = gamepad1.b;
        if (liftCollector) {

            if (collectorUp) { //the collector has been lifted
                collectordown = true;
                collectorUp = false;
            } else { //the collector has not been lifted
                collectorUp = true;
                collectordown = false;

            }

                if (collectordown) { //checking the power of the motors
                    robot.intake.setPower(0);//stop the motors
                    robot.shooter.setPower(0);

                }
                if (startInhaler && startWheel) { //we have to keep setting the power as long as startInhaler is true
                    robot.intake.setPower(powerMultiplier);
                    robot.shooter.setPower(powerMultiplier);
                }


            }
        }



            public void wobbleGoal () {
                boolean wobbleClawOpen = gamepad1.dpad_left;
                boolean wobbleClawClose = gamepad1.dpad_right;
                boolean wobbleArmUp = gamepad1.dpad_up;
                boolean wobbleArmDown = gamepad1.dpad_down;

                MAX_POS = this.robot.wobbleClaw.MAX_POSITION;
                MIN_POS = this.robot.wobbleClaw.MIN_POSITION;


                if (!wobbleArmUp && !wobbleArmDown) {

                    robot.wobbleArm.setPower(0);
                }

                if (wobbleArmUp) {
                    robot.wobbleArm.setPower(powerMultiplier);
                } else if (wobbleArmDown) {
                    robot.wobbleArm.setPower(-powerMultiplier);
                } else if (wobbleClawOpen) {
                    telemetry.addData("Claw open", wobbleClawPositon);
                    if (wobbleClawPositon <= MAX_POS) {
                        wobbleClawPositon += CLAWINCREMENT;
                    }
                    robot.wobbleClaw.setPosition(wobbleClawPositon);
                } else if (wobbleClawClose) {
                    telemetry.addData("Claw close", wobbleClawPositon);
                    if (wobbleClawPositon >= MIN_POS) {
                        wobbleClawPositon -= CLAWINCREMENT;
                        robot.wobbleClaw.setPosition(wobbleClawPositon);

                    }

                }
            }


}