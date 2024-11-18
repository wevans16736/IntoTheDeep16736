package org.firstinspires.ftc.teamcode.Autonomus;

// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

//Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//Team code imports
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotverticalslides.DriveActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalWristActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalIntakeActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalGrabberActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalWristActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;
//import org.firstinspires.ftc.teamcode.Autonomus.Configuration;


@Config
@Autonomous(name = "MainAuto", group = "Autonomus")
public class MainAutonomus extends LinearOpMode {

    private DriveActions driveActions = null;
    private HorizontalSlideActions horizontalSlide = null;
    private HorizontalWristActions horizontalWrist = null;
    private HorizontalIntakeActions horizontalIntake = null;
    private VerticalWristActions verticalWrist = null;
    private VerticalSlideActions verticalSlide = null;
    private VerticalGrabberActions verticalGrabber = null;

    public class VerticalSlideRR{
        public DcMotorEx verticalSlide1 = null;
        public DcMotorEx verticalSlide2 = null;


        public VerticalSlideRR(HardwareMap hardwareMap){
            verticalSlide1 = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE1);
            verticalSlide1.setDirection(DcMotorSimple.Direction.REVERSE);
            verticalSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide1.setTargetPosition(0);
            verticalSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            verticalSlide2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE2);
            verticalSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
            verticalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide2.setTargetPosition(0);
            verticalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        public class Liftup implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int position = -480;
                double velocity = 1800;
                if(!initialized){
                        verticalSlide1.setTargetPosition(position);
                        verticalSlide1.setVelocity(velocity);
                        verticalSlide2.setTargetPosition(-position);
                        verticalSlide2.setVelocity(-velocity);
                    initialized = true;
                }
                double currentPosition = verticalSlide1.getCurrentPosition();
                if (currentPosition > position){
                    return true;
                } else {
                    return false;

                }
            }
        }
        public Action liftUp(){
            return new Liftup();
        }
    }
    //make the whole class for the vertical wrist
    public class VerticalWristRR{
        public Servo verticalWristServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public VerticalWristRR(Telemetry opModeTelemetry, HardwareMap opModeHardware){
            telemetry = opModeTelemetry;
            hardwareMap = opModeHardware;
            verticalWristServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_WRIST);
            verticalWristServo.setPosition(0.8);
        }
        //this is a position to grab the butter from the wall or set it on the lower basket or either rung
        public class forwardDown implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalWristServo.setPosition(0.25);
                return false;
            }
        }
        public Action forwardDown(){
            return new forwardDown();
        }
        //this is a position to place it on the basket
        public class forwardUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                verticalWristServo.setPosition(0.4);
                return false;
            }
        }
        //this is a position to place it on the basket
        public Action forwardUp(){
            return new forwardUp();
        }
        //this is the position to grab the butter from the intake
        public class backwardPos implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalWristServo.setPosition(0.8);
                return false;
            }
        }
        public Action backwardPos(){
            return new backwardPos();
        }
    }
    //make the grabber method

    @Override
    public void runOpMode() throws InterruptedException {
        //set up all the hardware map for the robot and instantiate them as a object
        driveActions = new DriveActions(telemetry, hardwareMap);
        horizontalSlide = new HorizontalSlideActions(hardwareMap, telemetry);
        horizontalWrist = new HorizontalWristActions(telemetry, hardwareMap);
        horizontalIntake = new HorizontalIntakeActions(telemetry, hardwareMap);
        verticalSlide = new VerticalSlideActions(hardwareMap, telemetry);
        verticalWrist = new VerticalWristActions(telemetry, hardwareMap);
        verticalGrabber = new VerticalGrabberActions(telemetry, hardwareMap);
        VerticalSlideRR verticalSlideRR = new VerticalSlideRR(hardwareMap);
//        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
//                new TranslationalVelConstraint(50.0),
//                new AngularVelConstraint(Math.PI / 2)
//        ));

        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 25.0);

        //instantiate the robot to a particular pose.
        //todo find the correct initial position and put it below
        Pose2d initialPose = new Pose2d(0,0, Math.toRadians(90));
        Vector2d vector2d = new Vector2d(0,0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        //trajectory from initial spot moving to blue parking spot
        //todo find the correct blue park position and put it below
        
        TrajectoryActionBuilder parkBlue = drive.actionBuilder(initialPose)
                .lineToY(20);




        //trajectory from initial spot moving to red parking spot
//        TrajectoryActionBuilder parkRed = drive.actionBuilder(initialPose)
//                .splineTo(new Vector2d(20,30), Math.PI/2);


        //wait for the start button to be press
        waitForStart();
        //if stop button is press, automatically stop
        if (isStopRequested()) return;
        //choosing which trajectory to take, (so far only one is made)
        Action trajectoryActionChosen;
            trajectoryActionChosen = parkBlue.build();

        //run the chosen action blocking
        Actions.runBlocking(
            new SequentialAction(
                trajectoryActionChosen,
                    verticalSlideRR.liftUp()
            )

        );
    }
}



