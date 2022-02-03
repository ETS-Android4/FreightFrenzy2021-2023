package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;

public class DriveMecanum {

    private HWProfile robot;
    public double RF, LF, LR, RR;
    public LinearOpMode opMode;

    /*
     * Constructor method
     */
    public DriveMecanum(HWProfile myRobot, LinearOpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;
    }  // closes DriveMecanum constructor Method

    public void driveTime(double power, double heading, double duration) {
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;
        double theta = Math.toRadians(90 + heading);
        ElapsedTime runtime = new ElapsedTime();

        if(runtime.time() >= duration) active = false;

        updateValues(initZ, theta, currentZ, zCorrection);

        while(opMode.opModeIsActive() && active){
            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            if(runtime.time() >= duration) active = false;

            currentZ = getZAngle();
            if (currentZ != initZ){
//                zCorrection = Math.abs(initZ - currentZ)/100;
                zCorrection = 0;

                if (heading > 180 && heading < 359.999999) {
                    if (currentZ > initZ) {
                        RF = RF - zCorrection;
                        LF = LF + zCorrection;
                        LR = LR + zCorrection;
                        RR = RR - zCorrection;
                    } // end of if currentZ > initZ
                    if (currentZ < initZ) {
                        RF = RF + zCorrection;
                        LF = LF - zCorrection;
                        LR = LR - zCorrection;
                        RR = RR + zCorrection;
                    } // end of if currentZ < initZ
                }   // end of if heading > 180 && heading < 359.999999

                if (heading > 0 && heading < 180){
                    if(currentZ > initZ){
                        RF = RF + zCorrection;
                        LF = LF + zCorrection;
                        LR = LR - zCorrection;
                        RR = RR - zCorrection;
                    } // end of if currentZ > initZ
                    if (currentZ < initZ) {
                        RF = RF - zCorrection;
                        LF = LF - zCorrection;
                        LR = LR + zCorrection;
                        RR = RR + zCorrection;
                    } // end of if currentZ < initZ
                }   // end of if heading > 180 && heading < 359.999999

                if(heading == 0){
                    if(currentZ > initZ){
                        RF = RF - zCorrection;
                        LF = LF + zCorrection;
                        LR = LR + zCorrection;
                        RR = RR - zCorrection;
                    } // end of if currentZ > initZ
                    if (currentZ < initZ) {
                        RF = RF + zCorrection;
                        LF = LF - zCorrection;
                        LR = LR - zCorrection;
                        RR = RR + zCorrection;
                    } // end of if currentZ < initZ
                }   //end of if heading == 0

                if(heading == 180) {
                    if (currentZ > initZ) {
                        RF = RF + zCorrection;
                        LF = LF - zCorrection;
                        LR = LR - zCorrection;
                        RR = RR + zCorrection;
                    } // end of if currentZ > initZ
                    if (currentZ < initZ) {
                        RF = RF - zCorrection;
                        LF = LF + zCorrection;
                        LR = LR + zCorrection;
                        RR = RR - zCorrection;
                    } // end of if currentZ < initZ
                } // end of if heading == 180
            } // end of if current != initZ

            /*
             * Limit the value of the drive motors so that the power does not exceed 100%
             */
            if(RF > 1) RF = 1;
            else if (RF < -1) RF = -1;

            if(LF > 1) LF = 1;
            else if (LF < -1) LF = -1;

            if(LR > 1) LR = 1;
            else if (LR < -1) LR = -1;

            if(RR > 1) RR = 1;
            else if (RR < -1) RR = -1;

            /*
             * Apply power to the drive wheels
             */
            robot.motorRF.setPower(RF);
            robot.motorLF.setPower(LF);
            robot.motorLR.setPower(LR);
            robot.motorRR.setPower(RR);
        }   // end of while loop

        motorsHalt();
    }   // close robotCorrect method

    // distance will power side with highest power
    public void driveArcTurn(double powerLeft, double powerRight, double heading, double distance) {

    }

    public void driveTurn(double targetAngle, double errorFactor) {
        double integral = 0;
        int iterations = 0;
        ElapsedTime timeElapsed = new ElapsedTime();
        double startTime = timeElapsed.time();
        double totalTime;
        double v1, v2, v3, v4;
        double error=0;
        double Cp = 0.06;
        double Ci = 0.0003;
        double Cd = 0.0001;
        double maxSpeed = 0.8;
        double rotationSpeed;
        double derivative = 0, deltaError, lastError = 0;

        error = getZAngle() - targetAngle;
        // nested while loops are used to allow for a final check of an overshoot situation
        while ((Math.abs(error) >= errorFactor) && opMode.opModeIsActive()) {
            while ((Math.abs(error) >= errorFactor) && opMode.opModeIsActive()) {
                error = getZAngle() - targetAngle;
                deltaError = lastError - error;
                rotationSpeed = ((Cp * error) + (Ci * integral) + (Cd * derivative)) * maxSpeed;

                // Clip motor speed
                rotationSpeed = Range.clip(rotationSpeed, -maxSpeed, maxSpeed);

                if ((rotationSpeed > -0.21) && (rotationSpeed < 0)) {
                    rotationSpeed = -0.21;
                } else if ((rotationSpeed < 0.21) && (rotationSpeed > 0)) {
                    rotationSpeed = 0.21;
                }

                v1 = rotationSpeed;
                v2 = rotationSpeed;
                v3 = -rotationSpeed;
                v4 = -rotationSpeed;

                setDrivePower(v1,v2,v3,v4);

                lastError = error;
                iterations++;

                opMode.telemetry.addData("InitZ/targetAngle value  = ", targetAngle);
                opMode.telemetry.addData("Current Angle  = ", getZAngle());
                opMode.telemetry.addData("Theta/lastError Value= ", lastError);
                opMode.telemetry.addData("CurrentZ/Error Value = ", error);
                opMode.telemetry.addData("zCorrection/derivative Value = ", derivative);

                opMode.telemetry.addData("Right Front = ", v1);
                opMode.telemetry.addData("Left Front = ", v3);
                opMode.telemetry.addData("Left Rear = ", v4);
                opMode.telemetry.addData("Right Rear = ", v2);
                opMode.telemetry.update();


            }   // end of while Math.abs(error)
            motorsHalt();
            error = getZAngle() - targetAngle;
        }

        // shut off the drive motors
        motorsHalt();

        totalTime = timeElapsed.time() - startTime;
        opMode.telemetry.addData("Iterations = ", iterations);
        opMode.telemetry.addData("Final Angle = ", getZAngle());
        opMode.telemetry.addData("Total Time Elapsed = ", totalTime);
        opMode.telemetry.update();
    }   //end of the PIDRotate Method

    public void setDrivePower(double v1, double v2, double v3, double v4){
        robot.motorRF.setPower(v1);
        robot.motorRR.setPower(v2);
        robot.motorLF.setPower(v3);
        robot.motorLR.setPower(v4);
    }
    /*
     * Method getZAngle()
     */
    public double getZAngle(){
        return (-robot.imu.getAngularOrientation().firstAngle);
    }   // close getZAngle method

    /*
     * Method motorsHalt
     */
    public void  motorsHalt(){
        robot.motorRF.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    } // end of motorsHalt method

    public void motorsOn(double powerRF, double powerLF, double powerLR, double powerRR){
        robot.motorRF.setPower(powerRF);
        robot.motorLF.setPower(powerLF);
        robot.motorLR.setPower(powerLR);
        robot.motorRR.setPower(powerRR);
    }

    /*
     * method:      setArmLevel
     * Function:    sets the arm to the appropriate height for scoring in the hub
     * int level
     */
    public void setArmLevel(int level){
        double cupPosition = robot.INTAKECUPUP;
        int targetPosition = 0;

        if (level == 1){
            targetPosition = robot.ARMPOSITIONLEVEL1;
        } else if (level == 2) {
            targetPosition = robot.ARMPOSITIONLEVEL2;
        } else {
            targetPosition = robot.ARMPOSITIONLEVEL3;
        } // end of if (level ==1)

        robot.servoIntake.setPosition(cupPosition);
        robot.motorArm.setTargetPosition(targetPosition);
        robot.motorArm.setPower(-0.5);
        while (robot.motorArm.getCurrentPosition() > (targetPosition + 10)) {
            if (robot.motorArm.getCurrentPosition() > -5 &&
                    robot.motorArm.getCurrentPosition() < 2) {
                cupPosition = robot.INTAKECUPDOWN;
            }

            if (robot.motorArm.getCurrentPosition() < -7 &&
                    robot.motorArm.getCurrentPosition() > -200) {
                cupPosition = robot.INTAKECUPUP;
            }

            if (robot.motorArm.getCurrentPosition() < -300 &&
                    robot.motorArm.getCurrentPosition() > -800) {
                cupPosition = robot.INTAKECUPUP;
            }

            if (robot.motorArm.getCurrentPosition() < -800 &&
                    robot.motorArm.getCurrentPosition() > -1050) {
                cupPosition = robot.INTAKECUPINTERMED;
            }

            if (robot.motorArm.getCurrentPosition() < -1150 &&
                    robot.motorArm.getCurrentPosition() > -1200) {
                cupPosition = robot.INTAKECUPHIGH;
            }

            if (robot.motorArm.getCurrentPosition() < -1150 &&
                    robot.motorArm.getCurrentPosition() > -1200) {
                cupPosition = robot.INTAKECUPHIGH;
            }

            if (robot.motorArm.getCurrentPosition() < -1600) {
                cupPosition = robot.INTAKECUPSHARED;
            }
            robot.servoIntake.setPosition(cupPosition);
        }   // end of while(robot.motorArm...
    }   // end of method setLevelArm()

    /*
     * Method dumpCup
     */
    public void dumpCup(){
        robot.servoIntake.setPosition(robot.INTAKEHIGHDUMP);
        opMode.sleep(300);
        robot.servoIntake.setPosition(robot.INTAKECUPHIGH);
    }   // end of method dumpCup()

    public double tseDistance (){
        return robot.sensorDistance.getDistance(DistanceUnit.METER);
    }
    /*
     * Method resetArm()
     */
    public void resetArm() {
        double cupPosition = robot.INTAKECUPDOWN;
        int targetPosition = robot.ARMPOSITIONDOWN;
        robot.servoIntake.setPosition(cupPosition);
        robot.motorArm.setTargetPosition(targetPosition);
        robot.motorArm.setPower(-0.5);
        opMode.sleep(500);
    }

    /*
     * Method updateValues
     */
    public void updateValues(double initZ, double theta, double currentZ, double zCorrection){
        opMode.telemetry.addData("InitZ value = ", initZ);
        opMode.telemetry.addData("Theta Value = ", theta);
        opMode.telemetry.addData("Current Z value = ", currentZ);
        opMode.telemetry.addData("zCorrection Value = ", zCorrection);

        opMode.telemetry.addData("Right Front = ", RF);
        opMode.telemetry.addData("Left Front = ", LF);
        opMode.telemetry.addData("Left Rear = ", LR);
        opMode.telemetry.addData("Right Rear = ", RR);
        opMode.telemetry.update();
    }   // close updateValues method

}   // end of class DriveMecanum