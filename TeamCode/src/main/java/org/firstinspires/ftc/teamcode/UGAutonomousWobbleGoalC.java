package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="WobbleGoalC", group="Thunderbots")
public class UGAutonomousWobbleGoalC extends ThunderbotsSquareAutonomous{
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
        robot.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive1.getCurrentPosition(),
                robot.rightDrive1.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        parktargetline();
        // Step through each leg of the path

    }

    public void parktargetline() {

        double powerMultiplier = 0.5;
        double shooterPowerMultiplier = 0.5;

        robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

        encoderDrive(DRIVE_SPEED, 24, 24, 2.5);

        robot.wobbleArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        encoderDrive(DRIVE_SPEED,5,5, 0.7);

        double wobbleClawPosition = this.robot.wobbleClawServo.MAX_POSITION-1.5;
        robot.wobbleClawServo.setPosition(wobbleClawPosition);

        robot.wobbleArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderDrive(DRIVE_SPEED,5,5, 0.7);


        robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

        encoderDrive(DRIVE_SPEED, 24, 24, 0.9);

        robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);

        encoderDrive(DRIVE_SPEED, 24, 24, 1.9);

        robot.shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.shooterMotor.setPower(shooterPowerMultiplier);

        for (int a = 1; a<=3;a++) {

            double collectorPosition = this.robot.collectorServo.MAX_POSITION + 1.5;
            robot.collectorServo.setPosition(collectorPosition);
            sleep(1000);

            double triggerPosition = this.robot.triggerServo.MAX_POSITION - 1.5;
            robot.triggerServo.setPosition(triggerPosition);
            sleep(1000);
        }
        robot.shooterMotor.setPower(0);

        robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

        encoderDrive(DRIVE_SPEED, 24, 24, 0.8);




    }
}

