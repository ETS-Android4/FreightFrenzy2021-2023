package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;

@TeleOp(name = "Teleop Mode", group = "Competition")

public class MecanumTeleOp extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode(){
        double v1, v2, v3, v4, robotAngle;
        double theta;
        double theta2 = 180;
        double r;
        double rightX, rightY;
        boolean TSEFlag = false;
        boolean fieldCentric = false;
        int targetPosition = 0;
        double cupPosition = 0;

        ElapsedTime currentTime= new ElapsedTime();
        double buttonPress = currentTime.time();

        robot.init(hardwareMap);

        robot.motorArm.setTargetPosition(0);
        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        boolean shippingElement=false;
        boolean armDeployed=false;

        waitForStart();

        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAngularOrientation().firstAngle + 90;
            } else {
                theta = 0;      // do not adjust for the angular position of the robot
            }

            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            rightY = -gamepad1.right_stick_y;
            r = -Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

            v1 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);
            v2 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v3 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);
            v4 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);

            robot.motorLF.setPower(com.qualcomm.robotcore.util.Range.clip((v1), -1, 1));
            robot.motorRF.setPower(com.qualcomm.robotcore.util.Range.clip((v2), -1, 1));
            robot.motorLR.setPower(com.qualcomm.robotcore.util.Range.clip((v3), -1, 1));
            robot.motorRR.setPower(com.qualcomm.robotcore.util.Range.clip((v4), -1, 1));

            // Control which direction is forward and which is backward from the driver POV
            if (gamepad1.y && (currentTime.time() - buttonPress) > 0.3) {
                if (theta2 == 180) {
                    theta2 = 0;
                } else {
                    theta2 = 180;
                }
                buttonPress = currentTime.time();
            }   // end if (gamepad1.x && ...)

            if (gamepad1.left_bumper) {
                robot.motorDuck.setPower(robot.duckSpeed);
            } else if (gamepad1.right_bumper) {
                robot.motorDuck.setPower(-robot.duckSpeed);
            } else {
                robot.motorDuck.setPower(0);
            }   // end of if(gamepad1.left_bumper)

            /***********************
             * Arm control presets *
             ***********************/
            if(gamepad1.dpad_down || gamepad2.dpad_down) {
                targetPosition = robot.ARMPOSITIONDOWN;
                robot.servoIntake.setPosition(robot.INTAKECUPUP);
                shippingElement = false;
            } else if(gamepad1.dpad_right || gamepad2.dpad_right){
                // Go after the TSE
                targetPosition = robot.ARMPOSITIONSHARED;
            }else if(gamepad1.dpad_left  || gamepad2.dpad_left) {
                targetPosition = robot.ARMPOSITIONMID;
                shippingElement = false;
            } else if(gamepad1.dpad_up || gamepad2.dpad_up){
                targetPosition = robot.ARMPOSITIONHIGH;
            } else if (gamepad1.right_trigger > 0.5) {
                /***********************************************
                 ********* ARM CONTROL FOR TSE ACTIONS *********
                 ***********************************************/
                if(robot.motorArm.getCurrentPosition() > robot.ARMPOSITIONMID) {
                    targetPosition = robot.ARMPOSITIONTSELOW;
                } else {
                    if(robot.motorArm.getCurrentPosition() > robot.ARMPOSITIONTSELOW) {
                        // lower the arm
                        targetPosition = targetPosition - 20;
                    } else {
                        // limit how low the arm can go
                        targetPosition = robot.ARMPOSITIONTSELOW;
                    }
                }   // end of if(robot.motorArm.getCurrentPosition() >...
                shippingElement = true;
            } else if (gamepad1.left_trigger > 0.5){
                if(robot.motorArm.getCurrentPosition() > robot.ARMPOSITIONTSEHIGH) {
                    targetPosition = robot.ARMPOSITIONTSEHIGH;
                } else {
                    // raise the arm up in slow increments
                    targetPosition = targetPosition + 20;
                }   // end of if(robot.motorArm.getCurrentPosition() >...
                shippingElement = true;
            }   // end of if(gamepad1.dpad_down || gamepad2.dpad_down) with else statements

            /***********************************************
             ****** Arm controls for scoring elements ******
             ***********************************************/
            if(!shippingElement) {
                if (robot.motorArm.getCurrentPosition() > -5 &&
                        robot.motorArm.getCurrentPosition() < 2) {
                    cupPosition = robot.INTAKECUPDOWN;
                    telemetry.addData("currentPosition >armpositiondown - 5", "");
                }

                if (robot.motorArm.getCurrentPosition() < -7 &&
                        robot.motorArm.getCurrentPosition() > -200) {
                    cupPosition = robot.INTAKECUPUP;
                    telemetry.addData("Passing Intake", "");
                }

                if (robot.motorArm.getCurrentPosition() < -300 &&
                        robot.motorArm.getCurrentPosition() > -800) {
                    cupPosition = robot.INTAKECUPUP;
                    telemetry.addData("Passing Intake", "");
                }

                if (robot.motorArm.getCurrentPosition() < -800 &&
                        robot.motorArm.getCurrentPosition() > -1050) {
                    cupPosition = robot.INTAKECUPINTERMED;
                    telemetry.addData("currentPosition >armpositionIntermediate", "");
                }

                if (robot.motorArm.getCurrentPosition() < -1150 &&
                        robot.motorArm.getCurrentPosition() > -1200) {
                    cupPosition = robot.INTAKECUPHIGH;
                    telemetry.addData("currentPosition >armpositionHigh", "");
                }

                if (robot.motorArm.getCurrentPosition() < -1150 &&
                        robot.motorArm.getCurrentPosition() > -1410) {
                    cupPosition = robot.INTAKECUPHIGH;
                    telemetry.addData("currentPosition >armpositionHigh", "");
                }

                if (robot.motorArm.getCurrentPosition() < -1600) {
                    cupPosition = robot.INTAKECUPSHARED;
                    telemetry.addData("currentPosition >armpositionShared + 100", "");
                }
            }   // end of if(!shippingElement)

            /***********************************************
             ********* Arm controls for scoring TSE ********
             ***********************************************/
            if (shippingElement) {
                if (robot.motorArm.getCurrentPosition() > -5 &&
                        robot.motorArm.getCurrentPosition() < 2) {
                    cupPosition = robot.INTAKECUPTSEHIGH;
                    shippingElement = false;
                    telemetry.addData("currentPosition >armpositiondown - 5", "");
                }

                if (robot.motorArm.getCurrentPosition() < -1000 &&
                        robot.motorArm.getCurrentPosition() < -1600) {
//                    cupPosition = robot.INTAKECUPTSEHIGH;
                    telemetry.addData("currentPosition >armpositionTSEHigh + 100", "");
                }

                if (robot.motorArm.getCurrentPosition() < -1750) {
                    cupPosition = robot.INTAKECUPTSELOW;
                    telemetry.addData("currentPosition >armpositionTSE + 100", "");
                }

            }   // end of if (shippingElement)


            /***********************************************
             ********* Apply cup & arm Positioning  ********
             ***********************************************/

            if(gamepad1.b || gamepad2.b){
                cupPosition = robot.INTAKEHIGHDUMP; // dump the cup
            }

            robot.servoIntake.setPosition(cupPosition);
            sleep(50);
            robot.motorArm.setTargetPosition(targetPosition);
            robot.motorArm.setPower(-0.5);

            /***********************************************
             ********* INTAKE CONTROL  ********
             ***********************************************/
            if(gamepad1.a){
                robot.motorIntake.setPower(0.75);   // Turn intake on
            } else if (gamepad1.x || gamepad2.x) {
                robot.motorIntake.setPower(-0.75);  // reverse intake to clear extra SE
            } else {
                robot.motorIntake.setPower(0);  // turn intake off by default
            }

            // Provide user feedback
            telemetry.addData("V1 = ", v1);
            telemetry.addData("V2 = ", v2);
            telemetry.addData("V3 = ", v3);
            telemetry.addData("V4 = ", v4);
            telemetry.addData("Arm Motor Postion = ", robot.motorArm.getCurrentPosition());
            telemetry.addData("dpad_up = ", gamepad1.dpad_up);
            telemetry.addData("dpad_down = ", gamepad1.dpad_down);
            telemetry.addData("dpad_left = ", gamepad1.dpad_left);
            telemetry.addData("dpad_right = ", gamepad1.dpad_right);
            telemetry.addData("Left Stick X = ", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y = ", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X = ", gamepad1.right_stick_x);
            telemetry.addData("Right Stick Y = ", gamepad1.right_stick_y);
            telemetry.addData("Theta = ", theta);
            telemetry.addData("Theta2 = ", theta);
            telemetry.addData("IMU Value: ", theta);
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class