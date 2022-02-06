package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;

@TeleOp(name="robot drive", group="Dev")
@Disabled

public class TestDriveTeleOp extends LinearOpMode {
    /* Public OpMode members. */
    public HWProfile robot = new HWProfile();
    public double drive;
    public double turn;
    public double left;
    public double right;
    public double max;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            robot.motorLF.setPower(left);
            robot.motorLR.setPower(left);
            robot.motorRR.setPower(right);
            robot.motorRF.setPower(right);


            // Send telemetry message to signify robot running;
            telemetry.addData("left", "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
