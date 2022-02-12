package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@Autonomous(name = "Blue Duck - STATE", group = "Competition")

public class AutoBlueHubDuckv2 extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;
    private State state = State.DETECT_TSE;

    public AutoBlueHubDuckv2() {

    }   // end of TestAuto constructor

    public void runOpMode() {
        int hubLevel = 3;
        boolean isRunning = true;
        int forwardDistance = 0;
        ElapsedTime runtime = new ElapsedTime();
        double startTime = 0;

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

        while(!opModeIsActive() && isRunning){
            telemetry.addData("Side of Field => ", "CAROUSEL");
            telemetry.addData("Alliance      => ", "BLUE");
            telemetry.addData("Robot state   => ", "INITIALIZED");
            telemetry.addData("              => ", "");
            telemetry.addData("PRESS X       => ","TO ABORT PROGRAM");
            telemetry.addData("              => ","");
            telemetry.addData(" Z Value      => ", drive.getZAngle());
            telemetry.addData("Distance      => ", drive.tseDistance());
            telemetry.addData("              => ","");
            if (robot.sensorDistance.getDistance(DistanceUnit.CM) < robot.TSEDISTANCE) {
                telemetry.addData("TSE   => ", "DETECTED");
            } else {
                telemetry.addData("TSE   => ", "NOT DETECTED");
            }
            telemetry.update();

            if(gamepad1.x || gamepad2.x){
                isRunning = false;
                requestOpModeStop();
            }
        }   // end of while(!opModeIsActive() && isRunning)

        waitForStart();

        runtime.reset();
        startTime = runtime.time();

        while(opModeIsActive()) {
            switch (state) {
                case TEST:

                    break;

                case DETECT_TSE:
                    if(drive.tseDistance() < robot.TSEDISTANCE) {
                        hubLevel = 2;
                        // strafe to position in front of the hub
                        drive.driveTime(0.5, -90, 0.7);
                        forwardDistance = 12;       // how far to move forward to score
                    } else {
                        // strafe to the left away from the hub, stopping to check the next position
                        drive.driveTime(0.5, -90, 0.65);

                        // pause to allow time to determine if a TSE is present
                        sleep(300);
                        if(drive.tseDistance() < robot.TSEDISTANCE) {
                            hubLevel = 3;
                            forwardDistance = 14;       // how far to move forward to score
                        } else {
                            hubLevel = 1;
                            forwardDistance = 14;       // how far to move forward to score
                        } // end of if(drive.tseDistance()
                    } // end of if(drive.tseDistance() else

                    // drive forward to avoid hitting the wall
                    drive.driveStraight(-0.4, 4);

                    // raise the arm above the TSE to drive forward
                    robot.servoIntake.setPosition(robot.INTAKECUPUP);
                    robot.motorArm.setTargetPosition(robot.ARMPOSITIONMID);
                    robot.motorArm.setPower(0.5);
                    sleep(500);
                    telemetry.addData("Set arm to Level = ", "MID");
                    telemetry.update();
                    state = State.SCORE_TSE;

                case SCORE_TSE:
                    // forward into scoring position
                    drive.driveTime(0.7, 180, 1.15);

                    // turn towards hub
                    drive.driveTurn(-90, 0.3);

                    // set the arm to the correct height to place on the hub
                    drive.setArmLevel(hubLevel);

                    // drive forward to position to place the cube
                    drive.driveStraight(-0.4, forwardDistance);

                    // place the cube in the correct level
                    drive.dumpCup();
                    sleep(500); // wait for the block to dump

                    // return to the starting position
                    drive.driveTime(0.4, 0, 0.8);

                    // reset the arm to starting position
                    drive.resetArm();

                    state = State.RUN1;
                    break;

                case RUN1:
                    // turn to original position
                    drive.driveTurn(0, 0.3);

                    // reverse towards starting position
                    drive.driveTime(0.7, 0, 0.8);

                    // turn towards carousel
                    drive.driveTurn(-90, 0.3);

                    // strafe towards wall
                    drive.driveTime(0.6, 90, 1.3);

                    // strafe away from wall
                    drive.driveTime(0.5, -90, 0.55);

                    // drive towards carousel
                    drive.driveTime(0.5, 0, 0.7);

                    drive.motorsOn(-0.01, 0.01, -0.01, 0.01);

                    // turn carousel on
                    robot.motorDuck.setPower(0.4);
                    sleep(9000);
/*                    while((startTime - runtime.time()) < 27){
                        // do nothing
                    }

 */
                    robot.motorDuck.setPower(0);

                    // correct strafe angle
                    drive.driveTurn(-90, 0.3);

                    // strafe to storage
                    drive.driveTime(0.42, -90, 1.4);
                    drive.motorsHalt();

                    state = State.HALT;

                    break;

                case PARK:

                    state = State.HALT;

                    break;

                case HALT:

                    // Stop all motors
                    robot.motorDuck.setPower(0);
                    robot.motorArm.setPower(0);
                    robot.motorIntake.setPower(0);
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
        TEST, DETECT_TSE, SCORE_TSE, RUN1, PARK, HALT;
    }   // end of enum State

}   // end of class AutoBlueHubDuck