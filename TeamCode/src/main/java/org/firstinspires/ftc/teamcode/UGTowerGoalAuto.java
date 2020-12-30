package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Tower Goal Auto", group="Thunderbots")
public class UGTowerGoalAuto extends ThunderbotsSquareAutonomous {
    UGHardwarePushbot robot = new UGHardwarePushbot();
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        //initSkystoneCamera();
        // Send telemetry message to signify robot waiting;
        /*telemetry.addData("Status", "Init done");    //
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
                robot.rightDrive1.getCurrentPosition());*/
        telemetry.update();

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
}