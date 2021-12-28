package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@Autonomous(name = "Red Hub Double", group = "Competition")

public class AutoRedHubDouble extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;
    private State state = State.RUN1;

    public AutoRedHubDouble() {

    }   // end of TestAuto constructor

    public void runOpMode() {
        telemetry.addData("Robot State = ", "READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);

        /*
         * Initialize the drive class
         */
        DriveMecanum drive = new DriveMecanum(robot, opMode);

        /*
         * Calibrate / initialize the game sensor
         */

        robot.servoIntake.setPosition(robot.INTAKECUPUP);
        telemetry.addData("Z Value = ", drive.getZAngle());
        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            switch (state) {
                case TEST:

                    break;

                case PLACE_SE:

                    break;

                case RUN1:
                    // Pause for alliance partner
//                    sleep(5000);

                    // strafe towards hub
                    drive.driveTime(0.5, 90, 1);

                    // drive towards the hub
                    drive.driveTime(0.5, 180, 0.9);

                    sleep(500);

                    // move arm into scoring position
                    robot.motorArm.setTargetPosition(robot.ARMPOSITIONHIGH - 80);
                    robot.motorArm.setPower(0.4);

                    sleep(2000);

                    // return arm to stationary position
                    robot.motorArm.setTargetPosition(0);
                    robot.motorArm.setPower(0.4);

                    sleep(1000);

                    // drive towards wall
                    drive.driveTime(0.5, 0, 0.7);

                    // rotate towards warehouse
                    drive.driveTurn(-90, 0.3);

                    // strafe into the wall
                    drive.driveTime(0.5, 90, 1);

                    // drive towards warehouse
                    drive.driveTime(0.5, 0, 2.2);

                    state = State.BONUS;

                    break;

                case BONUS:
                    // turn on intake
                    robot.motorIntake.setPower(1);

                    // drive into the elements
                    drive.driveTime(0.4, 0, 1);

                    sleep(1000);

                    // assume elements captured
                    // set the cup to an upright position
                    robot.servoIntake.setPosition(robot.INTAKECUPUP);
                    sleep(50);
                    robot.motorArm.setTargetPosition(robot.ARMPOSITIONMID);
                    robot.motorArm.setPower(0.4);

                    sleep(100);

                    // set intake to spit out any scoring elements
                    robot.motorIntake.setPower(-1);

                    //rotate to -90
                    drive.driveTurn(-90, 0.3);

                    //strafe into the wall
                    drive.driveTime(0.5, 90, 0.5);

                    // drive to scoring position
                    drive.driveTime(0.5, 180, 2.5);

                    // turn to score
                    drive.driveTurn(-90, 0.3);

                    // drive towards the hub
                    drive.driveTime(0.5, 180, 0.9);

                    sleep(500);

                    // move arm into scoring position
                    robot.motorArm.setTargetPosition(robot.ARMPOSITIONHIGH - 80);
                    robot.motorArm.setPower(0.4);

                    sleep(1000);

                    // return arm to stationary position
                    robot.motorArm.setTargetPosition(0);
                    robot.motorArm.setPower(0.4);

                    // drive towards wall
                    drive.driveTime(0.5, 0, 0.7);

                    // rotate towards warehouse
                    drive.driveTurn(-90, 0.3);

                    // strafe into the wall
                    drive.driveTime(0.5, 90, 1);

                    // drive towards warehouse
                    drive.driveTime(0.5, 0, 2.2);

                    state = State.HALT;
                    break;

                case HALT:

                    // Stop all motors
                    drive.motorsHalt();

                    // End the program
                    requestOpModeStop();

                    break;
            }   // end of the switch state
        }   // end of if opModeIsActive()

        // End the program
        requestOpModeStop();

    }// end of runOpMode constructor

    enum State {
        TEST, PLACE_SE, RUN1, BONUS, PARK, HALT;
    }   // end of enum State

}   // end of class AutoBlueStorage