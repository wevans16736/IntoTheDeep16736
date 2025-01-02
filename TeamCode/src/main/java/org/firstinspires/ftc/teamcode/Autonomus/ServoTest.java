package org.firstinspires.ftc.teamcode.Autonomus;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.ConfigurationSecondRobot;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

@Config
@Autonomous(name="ServoTest", group = "SecondRobot")
public class ServoTest extends LinearOpMode {
    public static double VerticalWrist = ConfigurationSecondRobot.verticalWristIntake;
    public static double VerticalGrabber = ConfigurationSecondRobot.verticalClose;
    public static double HorizontalGrabber = ConfigurationSecondRobot.horizontalGrabberOpen;
    public static double HorizontalWrist = ConfigurationSecondRobot.horizontalWristIntake;
    public static double HorizontalRoll = ConfigurationSecondRobot.flat;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo VerticalWristServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_WRIST);
        Servo VerticalGrabberServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_GRABBER);

        Servo HorizontalGrabberServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_INTAKE);
        Servo HorizontalWristServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_WRIST);
        Servo HorizontalRollServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_ROLL);

        VerticalWristServo.setPosition(VerticalWrist);
        VerticalGrabberServo.setPosition(VerticalGrabber);

        HorizontalGrabberServo.setPosition(HorizontalGrabber);
        HorizontalWristServo.setPosition(HorizontalWrist);
        HorizontalRollServo.setPosition(HorizontalRoll);

        waitForStart();
    }
}
