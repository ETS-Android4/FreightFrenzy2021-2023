package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@Autonomous(name = "Red Warehouse Bonus", group = "Competition")

public class AutoRedHubDouble extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;
    private State state = State.DETECT_TSE;
    private boolean isRunning = true;
//    private State state = State.RUN1;
    private ElapsedTime runtime = new ElapsedTime();

    public AutoRedHubDouble() {

    }   // end of AutoRedHubDouble constructor

    public void runOpMode() {
        int hubLevel = 3;
        double forwardDistance =0;

        telemetry.addData("Setup for Warehouse ", "Warehouse Side");
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

        robot.servoIntake.setPosition(robot.INTAKECUPUP);
        telemetry.addData("Z Value = ", drive.getZAngle());
        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();

        while(!opModeIsActive() && isRunning){
            telemetry.addData("PRESS X => ","TO ABORT PROGRAM");
            if (robot.sensorDistance.getDistance(DistanceUnit.CM) < robot.TSEDISTANCE) {
                telemetry.addData("TSE = ", "DETECTED");
            } else {
                telemetry.addData("TSE = ", "NOT DETECTED");
            }
            telemetry.update();

            if(gamepad1.x || gamepad2.x){
                isRunning = false;
                requestOpModeStop();
            }
        }
        waitForStart();

        // reset the runtime clock
        runtime.reset();

        while(opModeIsActive()) {
            switch (state) {
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
                        drive.driveTime(0.4, 90, 0.5);
                    } // end of if(drive.tseDistance() else

                    drive.setArmLevel(hubLevel);
                    telemetry.addData("Set arm to Level = ", hubLevel);
                    telemetry.update();

                    // drive forward to position to place the cube
                    drive.driveStraight(0.4, forwardDistance);

                    // place the cube in the correct level
                    drive.dumpCup();
                    sleep(500); // wait for the block to dump

                    // return to the starting position
                    drive.driveStraight(0.4, (-forwardDistance + 3));

                    // reset the arm to starting position
                    drive.resetArm();

                    state = State.RUN1;
                    break;

                case RUN1:
                    // rotate towards warehouse
                    drive.driveTurn(-88, 0.6);

                    // strafe into the wall
                    drive.driveTime(0.8, 90, 0.5);

                    // rotate towards warehouse
                    drive.driveTurn(-88, 0.5);

                    // drive towards warehouse
                    drive.driveTime(0.7, -4, 1.4);

                    // lower the cup to intake more elements
                    robot.servoIntake.setPosition(robot.INTAKECUPDOWN);

                    if(opModeIsActive() && runtime.time() < 20) {
                        state = State.BONUS;
                    } else {
                        state = State.HALT;
                    }

                    break;

                case BONUS:
                    // turn on intake
                    robot.motorIntake.setPower(1);

                    // drive into the elements
                    drive.driveTime(0.7, -2, 0.7);

                    sleep(750);

                    // assume elements captured
                    // set the cup to an upright position
                    robot.servoIntake.setPosition(robot.INTAKECUPUP);
                    sleep(50);

                    // Lift arm up
                    robot.motorArm.setTargetPosition(robot.ARMPOSITIONMID);
                    robot.motorArm.setPower(0.55);

                    // set intake to spit out any scoring elements
                    robot.motorIntake.setPower(-1);

                    //rotate to -90
                    drive.driveTurn(-88, 0.4);

                    //strafe into the wall
                    drive.driveTime(0.7, 90, 0.4);

                    // drive to scoring position
                    drive.driveTime(0.9, 180, 1.4);

                    // turn off the intake
                    robot.motorIntake.setPower(0);

                    // turn to score
                    drive.driveTurn(0, 0.3);

                    // drive towards the hub
                    drive.driveTime(0.6, 180, 0.7);

                    // move arm into scoring position
                    robot.motorArm.setTargetPosition(robot.ARMPOSITIONHIGH - 80);
                    robot.motorArm.setPower(0.5);

                    sleep(1000);

                    // return arm to stationary position
                    robot.motorArm.setTargetPosition(0);
                    robot.motorArm.setPower(0.5);

                    // drive towards wall
                    drive.driveTime(0.8, 0, 0.6);

                    // rotate towards warehouse
                    drive.driveTurn(-88, 0.3);

                    // strafe into the wall
                    drive.driveTime(0.6, 90, 0.5);

                    // rotate towards warehouse
                    drive.driveTurn(-88, 0.3);

                    // turn on intake
                    robot.motorIntake.setPower(1);

                    // drive towards warehouse
                    drive.driveTime(0.8, -2, 1.5);

                    // lower the cup to intake more elements
                    robot.servoIntake.setPosition(robot.INTAKECUPDOWN);

                    if(runtime.time() > 25) {
                        state = State.HALT;
                    }
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
        TEST, DETECT_TSE, RUN1, BONUS, HALT
    }   // end of enum State

}   // end of class AutoRedHubDouble