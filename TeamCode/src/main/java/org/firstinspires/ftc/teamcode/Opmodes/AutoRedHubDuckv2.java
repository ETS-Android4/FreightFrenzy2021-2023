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
        int hubLevel = 3;
        int forwardDistance = 0;

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

                case DETECT_TSE:
                    if(drive.tseDistance() < robot.TSEDISTANCE) {
                        hubLevel = 2;
                        // strafe to position in front of the hub
                        drive.driveTime(0.4, 90, 1);
                        forwardDistance = 10;       // how far to move forward to score
                    } else {
                        // strafe to the left towards the hub, stopping to check the next position
                        drive.driveTime(0.4, 90, 0.5);

                        // pause to allow time to determine if a TSE is present
                        sleep(300);
                        if(drive.tseDistance() < robot.TSEDISTANCE) {
                            hubLevel = 3;
                            forwardDistance = 14;       // how far to move forward to score
                        } else {
                            hubLevel = 1;
                            forwardDistance = 10;       // how far to move forward to score
                        } // end of if(drive.tseDistance()

                        // strafe into position to place cube in the hub.
//                        drive.driveTime(0.4, -90, 0.5);
                    } // end of if(drive.tseDistance() else

                    drive.setArmLevel(hubLevel);
                    telemetry.addData("Set arm to Level = ", hubLevel);
                    telemetry.update();
                    state = State.SCORE_TSE;

                case SCORE_TSE:

                    // forward into scoring position
                    drive.driveTime(0.7, 180, 1.2);

                    // turn towards hub
                    drive.driveTurn(90, 0.3);

                    // drive forward to position to place the cube
                    drive.driveStraight(0.4, forwardDistance);

                    // place the cube in the correct level
                    drive.dumpCup();
                    sleep(500); // wait for the block to dump

                    // return to the starting position
                    drive.driveStraight(0.4, -forwardDistance);

                    // reset the arm to starting position
                    drive.resetArm();

                    state = State.RUN1;
                    break;

                case RUN1:
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

                    state = State.PARK;

                    break;

                case PARK:
                    // drive to storage
                    drive.driveTime(0.42, 180, 1.15);
                    drive.motorsHalt();

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
        TEST, DETECT_TSE, SCORE_TSE, RUN1, PARK, HALT;
    }   // end of enum State

}   // end of class AutoBlueStorage