package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@Autonomous(name = "Blue Warehouse - STATE", group = "Competition")
public class AutoBlueHubDouble extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;
    private State state = State.DETECT_TSE;
    private ElapsedTime runtime = new ElapsedTime();
    private DataLogger Dl;

    /*
     * Initialize the drive class
     */
    DriveMecanum drive = new DriveMecanum(robot, opMode);

    public AutoBlueHubDouble() {

    }   // end of TestAuto constructor

    public void runOpMode() {
        int hubLevel = 3;
        double forwardDistance =0;
        boolean isRunning = true;

        telemetry.addData("Robot State = ", "READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);


        while(!opModeIsActive() && isRunning){
            telemetry.addData("Side of Field => ", "WAREHOUSE");
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

        createDl();

        waitForStart();

        // reset the runtime clock
        runtime.reset();

        while(opModeIsActive()) {
            switch (state) {
                case TEST:

                    break;

                case DETECT_TSE:
                    if(drive.tseDistance() < robot.TSEDISTANCE) {
                        hubLevel = 2;
                        // strafe to position in front of the hub
                        drive.driveTime(0.6, -90, 1.1);
                        forwardDistance = 14;       // how far to move forward to score
                    } else {
                        // strafe to the left towards the hub, stopping to check the next position
                        drive.driveTime(0.6, -90, 0.6);

                        // pause to allow time to determine if a TSE is present
                        sleep(500);
                        if(drive.tseDistance() < robot.TSEDISTANCE) {
                            hubLevel = 3;
                            forwardDistance = 14;       // how far to move forward to score
                        } else {
                            hubLevel = 1;
                            forwardDistance = 14;       // how far to move forward to score
                        } // end of if(drive.tseDistance()

                        telemetry.addData("distance sensed = ", drive.tseDistance());
                        telemetry.addData("detected level = ", hubLevel);
                        telemetry.update();
//                        sleep(5000);
                        // strafe into position to place cube in the hub.
                        drive.driveTime(0.6, -90, 0.6);
                    } // end of if(drive.tseDistance() else

                    // drive forward to avoid hitting the wall
                    drive.driveStraight(-0.4, 4);

                    drive.setArmLevel(hubLevel);
                    telemetry.addData("Set arm to Level = ", hubLevel);
                    telemetry.update();

                    // drive forward to position to place the cube
                    drive.driveStraight(-0.4, forwardDistance);

                    // place the cube in the correct level
                    if (hubLevel == 1){
                        drive.dumpCupLow();
                    } else {
                        drive.dumpCup();
                    }
                    sleep(500); // wait for the block to dump

                    // return to the starting position
                    drive.driveTime(0.4, 0, 0.6);

                    // reset the arm to starting position
                    drive.resetArm();

                    state = State.RUN1;
                    break;

                case RUN1:
                    // rotate towards warehouse
                    drive.driveTurn(88, 0.6);

                    // strafe into the wall
                    drive.driveTime(0.8, -90, 0.5);

                    // rotate towards warehouse
                    drive.driveTurn(88, 0.5);

                    // drive towards warehouse
                    drive.driveTime(0.7, -3, 0.6);

                    // strafe into the wall
                    drive.driveTime(0.8, -90, 0.5);

                    // drive towards warehouse
                    drive.driveTime(0.7, -3, 1.2);

                    // lower the cup to intake more elements
                    robot.servoIntake.setPosition(robot.INTAKECUPDOWN);

                    if(opModeIsActive() && runtime.time() < 20) {
                        state = State.HALT;
                    } else {
                        state = State.HALT;
                    }

                    break;

                case BONUS:
                    // turn on intake
                    robot.motorIntake.setPower(1);

                    // drive into the elements
                    drive.driveTime(0.7, 2, 0.7);

                    sleep(500);

                    // assume elements captured
                    // set the cup to an upright position
                    robot.servoIntake.setPosition(0.8);
//                    robot.servoIntake.setPosition(robot.INTAKECUPUP);
                    sleep(100);

                    // Lift arm up
                    robot.motorArm.setTargetPosition(robot.ARMPOSITIONMID);
                    robot.motorArm.setPower(0.55);

                    // set intake to spit out any scoring elements
                    robot.motorIntake.setPower(-1);

                    //rotate to 90
                    drive.driveTurn(88, 0.4);

                    //strafe into the wall
                    drive.driveTime(0.7, -90, 0.4);

                    // drive to scoring position
                    drive.driveTime(0.9, 180, 1.5);

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
                    drive.driveTurn(88, 0.3);

                    // strafe into the wall
                    drive.driveTime(0.6, -90, 0.5);

                    // rotate towards warehouse
                    drive.driveTurn(88, 0.3);

                    // turn on intake
                    robot.motorIntake.setPower(1);

                    // drive towards warehouse
                    drive.driveTime(0.7, 2, 0.8);

                    // strafe into the wall
                    drive.driveTime(0.6, -90, 0.5);

                    // drive towards warehouse
                    drive.driveTime(0.7, 2, 0.8);

                    // lower the cup to intake more elements
                    robot.servoIntake.setPosition(robot.INTAKECUPDOWN);

                    if(runtime.time() > 20) {
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
        }   // end of while opModeIsActive()

        dlStop();               // stop the data logger

        // End the program
        requestOpModeStop();

    }// end of runOpMode constructor

    enum State {
        TEST, DETECT_TSE, RUN1, BONUS, HALT
    }   // end of enum State

    /**
     * Setup the dataLogger
     * The dataLogger takes a set of fields defined here and sets up the file on the Android device
     * to save them to.  We then log data later throughout the class.
     */
    public void createDl() {

        Dl = new DataLogger("AutoMecanumSimpleTest" + runtime.time());
        Dl.addField("runTime:       ");
        Dl.addField("Alliance:      ");
        Dl.addField("Gyro value:    ");
        Dl.addField("Dist. Sensor:  ");
        Dl.addField("LF Encoder:    ");
        Dl.addField("LR Encoder:    ");
        Dl.addField("RF Encoder:    ");
        Dl.addField("RR Encoder:    ");
        Dl.newLine();
    }

    /**
     * Log data to the file on the phone.
     */
    public void logData() {

        Dl.addField(String.valueOf(runtime.time()));
        Dl.addField("Blue Alliance");
        Dl.addField(String.valueOf(drive.getZAngle()));
        Dl.addField(String.valueOf(robot.sensorDistance.getDistance(DistanceUnit.CM)));
        Dl.addField(String.valueOf(robot.motorLF.getCurrentPosition()));
        Dl.addField(String.valueOf(robot.motorLR.getCurrentPosition()));
        Dl.addField(String.valueOf(robot.motorRF.getCurrentPosition()));
        Dl.addField(String.valueOf(robot.motorRR.getCurrentPosition()));
        Dl.newLine();
    }

    /**
     * Stop the DataLogger
     */
    private void dlStop() {
        Dl.closeDataLogger();
    }

}   // end of class AutoBlueStorage