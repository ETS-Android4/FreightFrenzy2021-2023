package org.firstinspires.ftc.teamcode.HWProfile;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HWProfile {
    /* Public OpMode members. */
    public DcMotor  motorLF   = null;
    public DcMotor  motorLR  = null;
    public DcMotor  motorRF     = null;
    public DcMotor  motorRR    = null;
    public DcMotor motorDuck = null;
    public DcMotor motorArm = null;
    public DcMotor motorIntake = null;
    public BNO055IMU imu = null;
    public Servo servoIntake = null;

    public final double duckSpeed=0.6;
    public final int autoSleepTime=5000;

    final public double INTAKECUPDOWN = 0.59;
    final public double INTAKECUPMID = 0.45;
    final public double INTAKECUPINTERMED = 0.30;
    final public double INTAKECUPHIGH = 0.17;
    final public double INTAKECUPSHARED = 0.0;
    final public double INTAKEHIGHDUMP= 0.65;
    final public double INTAKECUPUP= 0.74;
    final public int ARMPOSITIONDOWN = 0;
    final public int ARMPOSITIONMID = -490;
    final public int ARMPOSITIONHIGH = -1400;
    final public int ARMPOSITIONSHARED = -2000;


    final public double DISTANCEPERROTATION = 4; // assuming inches

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HWProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorLF = hwMap.get(DcMotor.class, "motorLF");
        motorLR = hwMap.get(DcMotor.class, "motorLR");
        motorRF = hwMap.get(DcMotor.class, "motorRF");
        motorRR = hwMap.get(DcMotor.class, "motorRR");
        motorDuck = hwMap.get(DcMotor.class, "motorDuck");
        motorArm = hwMap.get(DcMotor.class, "motorArm");
        motorIntake = hwMap.get(DcMotor.class, "motorIntake");
        motorLF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorLR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorRF.setDirection(DcMotor.Direction.FORWARD);
        motorRR.setDirection(DcMotor.Direction.FORWARD);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setTargetPosition(0);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRF.setPower(0);
        motorRR.setPower(0);
        motorDuck.setPower(0);
        motorIntake.setPower(0);
        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set all motors to zero power
        motorLF.setPower(0);
        motorLR.setPower(0);
        motorRF.setPower(0);
        motorRR.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoIntake = hwMap.get(Servo.class, "servoIntake");

        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

    }
 }  // end of HWProfile Class

