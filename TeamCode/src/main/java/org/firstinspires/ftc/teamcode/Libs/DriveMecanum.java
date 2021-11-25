package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    }  // closes DriveMechanum constructor Method

    public void driveTime(double power, double heading, double duration) {
        String action = "Initializing";
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;
        double theta = Math.toRadians(90 + heading);
        ElapsedTime runtime = new ElapsedTime();

        if(runtime.time() >= duration) active = false;

        updateValues(action, initZ, theta, currentZ, zCorrection);

        while(opMode.opModeIsActive() && active){
            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            if(runtime.time() >= duration) active = false;

            currentZ = getZAngle();
            if (currentZ != initZ){
                zCorrection = Math.abs(initZ - currentZ)/100;

                if (heading > 180 && heading < 359.999999) {
                    if (currentZ > initZ) {
                        RF = RF - zCorrection;
                        LF = LF + zCorrection;
                        LR = LR + zCorrection;
                        RR = RR - zCorrection;
                        action = "(heading > 180 && currentZ > initZ";
                    } // end of if currentZ > initZ
                    if (currentZ < initZ) {
                        RF = RF + zCorrection;
                        LF = LF - zCorrection;
                        LR = LR - zCorrection;
                        RR = RR + zCorrection;
                        action = "(heading < 180 && currentZ < initZ";
                    } // end of if currentZ < initZ
                }   // end of if heading > 180 && heading < 359.999999

                if (heading > 0 && heading < 180){
                    if(currentZ > initZ){
                        RF = RF + zCorrection;
                        LF = LF + zCorrection;
                        LR = LR - zCorrection;
                        RR = RR - zCorrection;
                        action = "(heading < 180 && currentZ > initZ";
                    } // end of if currentZ > initZ
                    if (currentZ < initZ) {
                        RF = RF - zCorrection;
                        LF = LF - zCorrection;
                        LR = LR + zCorrection;
                        RR = RR + zCorrection;
                        action = "(heading == 0 && currentZ < initZ";
                    } // end of if currentZ < initZ
                }   // end of if heading > 180 && heading < 359.999999

                if(heading == 0){
                    if(currentZ > initZ){
                        RF = RF - zCorrection;
                        LF = LF + zCorrection;
                        LR = LR + zCorrection;
                        RR = RR - zCorrection;
                        action = "(heading == 0 && currentZ > initZ";
                    } // end of if currentZ > initZ
                    if (currentZ < initZ) {
                        RF = RF + zCorrection;
                        LF = LF - zCorrection;
                        LR = LR - zCorrection;
                        RR = RR + zCorrection;
                        action = "(heading < 180 && currentZ < initZ";
                    } // end of if currentZ < initZ
                }   //end of if heading == 0


                if(heading == 180) {
                    if (currentZ > initZ) {
                        RF = RF + zCorrection;
                        LF = LF - zCorrection;
                        LR = LR - zCorrection;
                        RR = RR + zCorrection;
                        action = "(heading == 0 && currentZ > initZ";
                    } // end of if currentZ > initZ
                    if (currentZ < initZ) {
                        RF = RF - zCorrection;
                        LF = LF + zCorrection;
                        LR = LR + zCorrection;
                        RR = RR - zCorrection;
                        action = "(heading < 180 && currentZ < initZ";
                    } // end of if currentZ < initZ
                } // end of if heading == 180
            } // end of if current != initZ

            /*
             * Limit the value of the drive motors so that the power does not acceed 100%
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


    public void driveDistance(double power, double heading, double distance) {
        String action = "Initializing";
        double initZ = getZAngle();     //calls gyro for value; keeps value constant
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;      // a boolean is true or false
        double theta = Math.toRadians(90 + heading);
        ElapsedTime runtime = new ElapsedTime();

        double  avgTicks = (robot.motorRF.getCurrentPosition()
                + robot.motorLF.getCurrentPosition()
                + robot.motorLR.getCurrentPosition()
                + robot.motorRR.getCurrentPosition() ) /4;
        // take avg. ticks and divide by ticksPerRotation to get number of rotations
        // multiply number of rotations by the circumference of the wheel to get the distance traveled

        double rotations = (avgTicks / robot.TICKSPERROTATION);
        double distanceTraveled = (rotations * robot.DISTANCEPERROTATION); //circumference of wheel is equal to DISTANCEPERROTATION

        if(distanceTraveled >= distance) active = false;

        updateValues(action, initZ, theta, currentZ, zCorrection);

        while(opMode.opModeIsActive() && active){
            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            avgTicks = (robot.motorRF.getCurrentPosition()
                    + robot.motorLF.getCurrentPosition()
                    + robot.motorLR.getCurrentPosition()
                    + robot.motorRR.getCurrentPosition() ) /4;
            // take avg. ticks and divide by ticksPerRotation to get number of rotations
            // multiply number of rotations by the circumference of the wheel to get the distance traveled

            rotations = (avgTicks / robot.TICKSPERROTATION);
            distanceTraveled = (rotations * robot.DISTANCEPERROTATION); //circumference of wheel is equal to DISTANCEPERROTATION

            if(distanceTraveled >= distance) active = false;avgTicks = (robot.motorRF.getCurrentPosition()
                    + robot.motorLF.getCurrentPosition()
                    + robot.motorLR.getCurrentPosition()
                    + robot.motorRR.getCurrentPosition() ) /4;
            // take avg. ticks and divide by ticksPerRotation to get number of rotations
            // multiply number of rotations by the circumference of the wheel to get the distance traveled

            rotations = (avgTicks / robot.TICKSPERROTATION);
            distanceTraveled = (rotations * robot.DISTANCEPERROTATION); //circumference of wheel is equal to DISTANCEPERROTATION

            if(distanceTraveled >= distance) active = false;



            currentZ = getZAngle();
            if (currentZ != initZ){
                zCorrection = Math.abs(initZ - currentZ)/100;

                if (heading > 180 && heading < 359.999999) {
                    if (currentZ > initZ) {
                        RF = RF - zCorrection;
                        LF = LF + zCorrection;
                        LR = LR + zCorrection;
                        RR = RR - zCorrection;
                        action = "(heading > 180 && currentZ > initZ";
                    } // end of if currentZ > initZ
                    if (currentZ < initZ) {
                        RF = RF + zCorrection;
                        LF = LF - zCorrection;
                        LR = LR - zCorrection;
                        RR = RR + zCorrection;
                        action = "(heading < 180 && currentZ < initZ";
                    } // end of if currentZ < initZ
                }   // end of if heading > 180 && heading < 359.999999

                if (heading > 0 && heading < 180){
                    if(currentZ > initZ){
                        RF = RF + zCorrection;
                        LF = LF + zCorrection;
                        LR = LR - zCorrection;
                        RR = RR - zCorrection;
                        action = "(heading < 180 && currentZ > initZ";
                    } // end of if currentZ > initZ
                    if (currentZ < initZ) {
                        RF = RF - zCorrection;
                        LF = LF - zCorrection;
                        LR = LR + zCorrection;
                        RR = RR + zCorrection;
                        action = "(heading == 0 && currentZ < initZ";
                    } // end of if currentZ < initZ
                }   // end of if heading > 180 && heading < 359.999999

                if(heading == 0){
                    if(currentZ > initZ){
                        RF = RF - zCorrection;
                        LF = LF + zCorrection;
                        LR = LR + zCorrection;
                        RR = RR - zCorrection;
                        action = "(heading == 0 && currentZ > initZ";
                    } // end of if currentZ > initZ
                    if (currentZ < initZ) {
                        RF = RF + zCorrection;
                        LF = LF - zCorrection;
                        LR = LR - zCorrection;
                        RR = RR + zCorrection;
                        action = "(heading < 180 && currentZ < initZ";
                    } // end of if currentZ < initZ
                }   //end of if heading == 0


                if(heading == 180) {
                    if (currentZ > initZ) {
                        RF = RF + zCorrection;
                        LF = LF - zCorrection;
                        LR = LR - zCorrection;
                        RR = RR + zCorrection;
                        action = "(heading == 0 && currentZ > initZ";
                    } // end of if currentZ > initZ
                    if (currentZ < initZ) {
                        RF = RF - zCorrection;
                        LF = LF + zCorrection;
                        LR = LR + zCorrection;
                        RR = RR - zCorrection;
                        action = "(heading < 180 && currentZ < initZ";
                    } // end of if currentZ < initZ
                } // end of if heading == 180
            } // end of if current != initZ

            /*
             * Limit the value of the drive motors so that the power does not acceed 100%
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


    /*
     * Method updateValues
     */
    public void updateValues(String action, double initZ, double theta, double currentZ, double zCorrection){
        opMode.telemetry.addData("Current Action = ", action);
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

}