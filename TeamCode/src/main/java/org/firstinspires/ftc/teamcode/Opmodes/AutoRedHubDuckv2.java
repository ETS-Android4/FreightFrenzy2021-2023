package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@Autonomous(name = "Red Hub Duck v2", group = "Competition")

public class AutoRedHubDuckv2 extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;
    private State state = State.RUN1;

    public AutoRedHubDuckv2() {

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
                    // forward into scoring position
                    drive.driveTime(0.7, 180, 1.2);

                    // turn towards hub
                    drive.driveTurn(90, 0.3);

                    // drive towards hub
                    drive.driveTime(0.5, 180, 0.4);

                    // score in the hub
                    robot.motorArm.setTargetPosition(robot.ARMPOSITIONHIGH - 80);
                    robot.motorArm.setPower(0.4);

                    sleep(2000);

                    // reset the arm to normal position
                    robot.motorArm.setTargetPosition(0);
                    robot.motorArm.setPower(0.4);

                    sleep(1000);

                    // turn to original position
                    drive.driveTurn(0, 0.3);

                    // strafe towards wall
                    drive.driveTime(0.52, 90, 1.2);


                    // drive towards carousel
                    drive.driveTime(0.5, 0, 1.7);
                    drive.motorsOn(-0.05, 0.05, -0.05, 0.05);

                    // turn carousel on
                    robot.motorDuck.setPower(-0.4);
                    sleep(10000);
                    robot.motorDuck.setPower(0);

                    // drive to storage
                    drive.driveTime(0.42, 180, 1.15);
                    drive.motorsHalt();

                    state = State.HALT;

                    break;

                case PARK:

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

    }// end of runOpMode constructor

    enum State {
        TEST, PLACE_SE, RUN1, PARK, HALT;
    }   // end of enum State

}   // end of class AutoBlueStorage