package org.firstinspires.ftc.teamcode.secondrobot.constants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorConstants {

    public static final int DIRECTION_FORWARD = 0;
    public static final int DIRECTION_REVERSE = 1;
    public static final int DIRECTION_STRAFE_RIGHT = 2;
    public static final int DIRECTION_STRAFE_LEFT = 3;

    public static final int DIRECTION_UP = 4;
    public static final int DIRECTION_DOWN = 5;


    // Added for simplicity
    public static final DcMotorSimple.Direction REVERSE = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FORWARD = DcMotorSimple.Direction.FORWARD;
    public static final DcMotor.ZeroPowerBehavior BRAKE = DcMotor.ZeroPowerBehavior.BRAKE;


    // Ticks per millimeter of robot travel is:
    // (7x4x2x13.7) / (96 * pi) = 2.5438 ticks/mm
    // (poles per encoder magnet) * (number of edges) * (gear ratio of bevel gears) * (gear ratio) / (wheel circumference)


    public static final double TICKS_PER_MM = 2.5438;
    public static final double TICKS_PER_INCH = 64.71;

    public static final double MAX_VELOCITY_WHEELS = 2800;

    // You can have the method take a parameter of distance, and convert that to encoder ticks.

    // 5202 Series Yellow Jacket Planetary Gear Motor (13.7:1 Ratio, 435 RPM, 3.3 - 5V Encoder)
    // Encoder Countable Events Per Revolution (Output Shaft)	383.6 (Rises & Falls of Ch A & B)
    public static final double COUNTS_PER_MOTOR_REVOLUTION = 386.3;    // eg: GOBILDA Motor Encoder, 435 RPM
    public static final double WHEEL_CIRCUMFERENCE_INCHES = 12.37;     // C = 2 * 3.14 * r
    public static final double DISTANCE_PER_REVOLUTION_INCHES = 12.37;     // For figuring circumference
    public static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REVOLUTION / DISTANCE_PER_REVOLUTION_INCHES;   //31.22f

    // Mecanum wheels are 100 mm diameter = 3.94 inches
    // Circumference = 2 * 3.14 * r = 2 * 3.14 * 50mm = 12.37 inches; Distance travelled per rotation is same as the circumference
    // Wheel uses 386.3 ticks and travels 12.37 inches; This is 31.23 ticks per Inch

    //Encoder FORWARD 1 inch
    public static final float ENCODER_CLICKS_FORWARD_1_INCH = 31.22f;   //COUNTS_PER_INCH

    //Encoder STRAFE 1 inch
    public static final float ENCODER_CLICKS_STRAFE_1_INCH = 25.8944908f;

}
