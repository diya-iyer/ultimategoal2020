package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Power Goal Auto", group="Thunderbots")
public class UGStraightShooter extends ThunderbotsSquareAutonomous {
    UGHardwarePushbot robot = new UGHardwarePushbot();
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
        robot.leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive1.getCurrentPosition(),
                robot.rightDrive1.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        gettotargetline();
        strafelefttogoalpost();
        shoot();

        moveforwardandpark();

            // Step through each leg of the path

    }

    public void gettotargetline() {
        //moveforwardtolaunchline

        double powerMultiplier = 0.5;

        robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

        encoderDrive(DRIVE_SPEED, 24, 24, 1.8);

    }

    public void strafelefttogoalpost() {
        //robot gets into position to shoot the power goals//

        double powerMultiplier = 2;

        robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);

        encoderDrive(DRIVE_SPEED, 24, 24, 1.0);


    }
    public void shoot() {
        double shooterPowerMultiplier = 0.5;

        robot.shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.shooter.setPower(shooterPowerMultiplier);

        for (int a = 1; a<=3;a++) {

            double collectorPosition = this.robot.collector.MAX_POSITION + 1.5;
            robot.collector.setPosition(collectorPosition);

            double triggerPosition = this.robot.trigger.MAX_POSITION - 1.5;
            robot.trigger.setPosition(triggerPosition);

        }
        robot.shooter.setPower(0);


    }

    public void moveforwardandpark() {


        double powerMultiplier = 0.5;

        robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

        encoderDrive(DRIVE_SPEED, 24, 24, 0.3);


            }
        }

