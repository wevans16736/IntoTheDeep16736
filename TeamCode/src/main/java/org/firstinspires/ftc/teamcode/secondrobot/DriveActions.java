package org.firstinspires.ftc.teamcode.secondrobot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

/**
 * Make sure to have the following:
 *
 * 1. Hardware config
 * 2. Setup direction of motors
 * 3. Action method to do something (hookUpDown, drive, etc.,)
 * 4. Helper methods (stop, brake, leftTurn, rightTurn, etc.,)
 *
 * Purpose: Drive the 4 wheels
 */
public class DriveActions {

    public DcMotorEx leftFront;
    public DcMotorEx leftRear;

    public DcMotorEx rightFront;
    public DcMotorEx rightRear;

    public IMU imu;
    //the amount to throttle the power of the motors
    public double THROTTLE = 1.5;

    private double robotHeading = 0;
    public double headingOffset = 0;
    private double headingError = 0;
    private double targetHeading = 0;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();

    public boolean applySensorSpeed = false;

    /**
     * Creates a mecanum motor using the 4 individual motors passed in as the arguments
     * @param opModeTelemetry : Telemetry to send messages to the Driver Control
     * @param opModeHardware : Hardware Mappings
     */
    // Constructor
    public DriveActions(Telemetry opModeTelemetry, HardwareMap opModeHardware ) {

        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;

        // 1. Hardware config
        leftFront = hardwareMap.get(DcMotorEx.class, ConfigConstants.FRONT_LEFT);
        leftRear = hardwareMap.get(DcMotorEx.class, ConfigConstants.BACK_LEFT);

        rightFront = hardwareMap.get(DcMotorEx.class, ConfigConstants.FRONT_RIGHT);
        rightRear = hardwareMap.get(DcMotorEx.class, ConfigConstants.BACK_RIGHT);

        // old code
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        rightFront.setVelocity(0.0);
//        leftFront.setVelocity(0.0);
//        rightRear.setVelocity(0.0);
//        leftRear.setVelocity(0.0);


        rightFront.setPower(0.0);
        leftFront.setPower(0.0);
        rightRear.setPower(0.0);
        leftRear.setPower(0.0);

//        rightFront.setVelocity(0.0);
//        leftFront.setVelocity(0.0);
//        rightRear.setVelocity(0.0);
//        leftRear.setVelocity(0.0);

        // 2. Set direction
        setMotorDirection_Forward();

        //IMU Setup
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        resetHeading();
    }

    public void setSpeed(double mySpeed){
        THROTTLE = mySpeed;
    }


    /**
     * Drive method to throttle the power
     * @param speedX - the x value of the joystick controlling strafe
     * @param speedY - the y value of the joystick controlling the forward/backward motion
     * @param rotation - the x value of the joystick controlling the rotation
     */
    public void drive(double speedX, double speedY, double rotation){

        if (leftFront.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER &!GlobalVariables.driveDisable) {
            double throttledX = speedX * THROTTLE;
            double throttledY = speedY * THROTTLE;
            double throttledRotation = rotation * THROTTLE;

            driveUsingJoyStick(throttledX, throttledY, throttledRotation);
        }
    }

    /**
     * This function makes the mecanum motor drive using the joystick
     * @param speedX - the x value of the joystick controlling strafe
     * @param speedY - the y value of the joystick controlling the forward/backwards motion
     * @param rotation - the x value of the joystick controlling the rotation
     */
    public void driveUsingJoyStick(double speedX, double speedY, double rotation) {

        double frontLeft = speedX + speedY - rotation;
        double frontRight = -speedX + speedY + rotation;

        double backLeft = -speedX + speedY - rotation;
        double backRight = speedX + speedY + rotation;

//        double fl = speedX + speedY + rotation;
//        double fr = -speedX + speedY - rotation;
//        double bl= -speedX + speedY + rotation;
//        double br = speedX + speedY - rotation;

        double max = getMaxPower(frontLeft, frontRight, backLeft, backRight);
        if (max > 1) {
            frontLeft = frontLeft / max;
            frontRight = frontRight / max;
            backLeft = backLeft / max;
            backRight = backRight / max;
        }
        // old code
        rightFront.setPower(frontRight);
        leftFront.setPower(frontLeft);
        rightRear.setPower(backRight);
        leftRear.setPower(backLeft);




//        rightFront.setVelocity((frontRight * MotorConstants.MAX_VELOCITY_WHEELS));
//        leftFront.setVelocity((frontLeft * MotorConstants.MAX_VELOCITY_WHEELS));
//        rightRear.setVelocity((backRight * MotorConstants.MAX_VELOCITY_WHEELS));
//        leftRear.setVelocity((backLeft * MotorConstants.MAX_VELOCITY_WHEELS));

    }

    private double getMaxPower(double frontLeftValue, double frontRightValue, double backLeftValue, double backRightValue) {
        List<Double> valueList = new LinkedList<>();
        valueList.add(frontLeftValue);
        valueList.add(frontRightValue);
        valueList.add(backLeftValue);
        valueList.add(backRightValue);

        return Collections.max(valueList);
    }

    public void setPowerMax() {
        rightFront.setPower(1.0);
        leftFront.setPower(1.0);
        rightRear.setPower(1.0);
        leftRear.setPower(1.0);
    }

    //This methods is meant for AUTONOMOUS
    public void setMotorDirection_Forward() {
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        return getSteeringCorrection(desiredHeading, proportionalGain, 1);
    }
    public double getSteeringCorrection(double desiredHeading, double proportionalGain, double range) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading();
        RobotLog.dd("Gyro", "Heading %f", robotHeading);
//        telemetry.addData("robotHeading", robotHeading);
//        telemetry.update();
//        opModeObj.sleep(3000);

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -range, range);
    }

    public double getRawHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES) - headingOffset;
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        headingOffset = orientation.getYaw(AngleUnit.DEGREES);
        robotHeading = 0;
    }
    public void strafeDistance(double distance) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double ticksPerCM = 12;

        int totalTicks = (int) (ticksPerCM * distance);
        // When going left, the target position needs to be inverted from when we go right
        leftFront.setTargetPosition(-totalTicks);
        rightFront.setTargetPosition(totalTicks);
        leftRear.setTargetPosition(totalTicks);
        rightRear.setTargetPosition(-totalTicks);

        // Switch to RUN_TO_POSITION mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double speed = 2000;
        leftFront.setVelocity(-speed);
        rightFront.setVelocity(-speed);
        leftRear.setVelocity(-speed);
        rightRear.setVelocity(-speed);
    }
    boolean isDone() {
        return leftFront.isBusy();
    }
    void runWithoutEncoders() {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}