package org.firstinspires.ftc.teamcode.Autonomus;

// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

@Config
@Autonomous(name = "Final Auto", group = "Autonomous")
public class FinalAuto extends LinearOpMode {
    public class VerticalSlideRR {
        public DcMotorEx verticalSlide1 = null;
        public DcMotorEx verticalSlide2 = null;

        public VerticalSlideRR(HardwareMap hardwareMap) {
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
        private boolean initialized = false;
        public class VerticalSlidePosition implements Action {
            int position = 0;
            private boolean initialized = false;

            public VerticalSlidePosition(int position) {
                this.position = position;
            }

            public VerticalSlidePosition() {
                this.position = Configuration.bottom;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double velocity = 1800;
                if (!initialized) {
                    verticalSlide1.setTargetPosition(position);
                    verticalSlide1.setVelocity(velocity);
                    verticalSlide2.setTargetPosition(-position);
                    verticalSlide2.setVelocity(-velocity);
                    initialized = true;
                }
                double currentPosition = verticalSlide1.getCurrentPosition();
                return currentPosition > position;
            }
        }
        public Action verticalSlidePosition() {
            return new VerticalSlidePosition();}
        public Action verticalSlidePosition(int position) {
            return new VerticalSlidePosition(position);
        }
    }

    public class HorizontalSlideRR {
        public DcMotorEx horizontalSlide2 = null;
        private Telemetry telemetry;

        public HorizontalSlideRR(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;
            horizontalSlide2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.HORIZONTAL_SLIDE2);
            horizontalSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
            horizontalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalSlide2.setTargetPosition(0);
            horizontalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        private boolean initialized = false;

        public class HorizontalSLidePosition implements Action {
            int position = Configuration.retractSlide;
            private boolean initialized = false;

            public HorizontalSLidePosition(int position) {
                this.position = position;
            }

            public HorizontalSLidePosition() {
                this.position = Configuration.retractSlide;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double velocity = 1800;
                if (!initialized) {
                    horizontalSlide2.setTargetPosition(position);
                    horizontalSlide2.setVelocity(velocity);
                    horizontalSlide2.setTargetPosition(-position);
                    horizontalSlide2.setVelocity(-velocity);
                    initialized = true;
                }
                double currentPosition = horizontalSlide2.getCurrentPosition();
                return currentPosition > position;
            }
        }
        public Action horizontalSlidePosition(int position) {
            return new HorizontalSLidePosition(position);
        }
        public Action horizontalSlidePosition() {
            return new HorizontalSLidePosition();
        }
    }

    public class VerticalGrabberRR {
        public Servo verticalGrabberServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public VerticalGrabberRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            verticalGrabberServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_GRABBER);
            verticalGrabberServo.setPosition(1.0);
        }

        public class VerticalGrabberPosition implements Action {
            double position = Configuration.close;
            public VerticalGrabberPosition(double position){
                this.position = position;
            }
            public VerticalGrabberPosition(){
                this.position = Configuration.close;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                verticalGrabberServo.setPosition(position);
                return false;
            }
        }
        public Action verticalGrabberPosition(double position) {return new VerticalGrabberPosition(position);}
        public Action verticalGrabberPosition() {return new VerticalGrabberPosition();}
    }
    public class VerticalWristRR{
        public Servo verticalWristServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public VerticalWristRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            verticalWristServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_WRIST);
            verticalWristServo.setPosition(Configuration.backwardPos);
        }
        public class VerticalWristPosition implements Action{
            double position = Configuration.backwardPos;
            public VerticalWristPosition(double position){
                this.position = position;
            }
            public VerticalWristPosition(){
                this.position = Configuration.backwardPos;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalWristServo.setPosition(position);
                return false;
            }
        }
        public Action verticalWristPosition(double position) {return new VerticalWristPosition(position);}
        public Action verticalWristPosition() {return  new VerticalWristPosition();}
    }
    public class HorizontalGrabberRR{
        public Servo intakeServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public HorizontalGrabberRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            intakeServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_INTAKE);
            intakeServo.setPosition(Configuration.floorClose);
        }
        public class HorizontalIntakePosition implements Action{
            double position = Configuration.floorClose;
            public HorizontalIntakePosition(double position){
                this.position = position;
            }
            public HorizontalIntakePosition(){
                this.position = Configuration.floorClose;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intakeServo.setPosition(position);
                return false;
            }
        }
        public Action horizontalIntakePosition(double position) {return new HorizontalIntakePosition(position);}
        public Action horizontalIntakePosition() {return new HorizontalIntakePosition();}
    }
    public class HorizontalWristRR{
        public Servo horizontalWristServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public HorizontalWristRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            horizontalWristServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_WRIST);
            horizontalWristServo.setPosition(Configuration.backwardPosIn);
        }
        public class HorizontalWristPosition implements Action{
            double position = Configuration.backwardPosIn;
            public HorizontalWristPosition(double position){
                this.position = position;
            }
            public HorizontalWristPosition(){
                this.position = Configuration.backwardPosIn;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                horizontalWristServo.setPosition(position);
                return false;
            }
        }
        public Action horizontalWristPosition(double position) {return new HorizontalWristPosition(position);}
        public Action horizontalWristPosition() {return new HorizontalWristPosition();}
    }
    public class HorizontalRollRR{
        public Servo rollServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public HorizontalRollRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            rollServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_ROLL);
            rollServo.setPosition(Configuration.flat);
        }
        public class HorizontalRollPosition implements Action{
            double position = Configuration.flat;
            public HorizontalRollPosition(double position){
                this.position = position;
            }
            public HorizontalRollPosition(){
                this.position = Configuration.flat;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                rollServo.setPosition(position);
                return false;
            }
        }
        public Action horizontalRollPosition(double position) {return new HorizontalRollPosition(position);}
        public Action horizontalRollPosition() {return new HorizontalRollPosition();}
    }

    @Override
    public void runOpMode() throws InterruptedException {
        VerticalSlideRR verticalSlideRR = new VerticalSlideRR(hardwareMap);
        HorizontalSlideRR horizontalSlideRR = new HorizontalSlideRR(hardwareMap, telemetry);

        VerticalGrabberRR verticalGrabberRR = new VerticalGrabberRR(hardwareMap, telemetry);
        HorizontalGrabberRR horizontalIntakeRR = new HorizontalGrabberRR(hardwareMap, telemetry);

        VerticalWristRR verticalWristRR = new VerticalWristRR(hardwareMap, telemetry);
        HorizontalWristRR horizontalWristRR = new HorizontalWristRR(hardwareMap, telemetry);

        HorizontalRollRR horizontalRollRR = new HorizontalRollRR(hardwareMap, telemetry);

        Pose2d currentPose = new Pose2d(0,0,Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        Actions.runBlocking(new SequentialAction(
                horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide),
                verticalSlideRR.verticalSlidePosition(Configuration.bottom),
                horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose),
                verticalGrabberRR.verticalGrabberPosition(Configuration.close),
                horizontalWristRR.horizontalWristPosition(.7),
                verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                horizontalRollRR.horizontalRollPosition(Configuration.flat)
        ));

        boolean pickRight = false;
        boolean pickLeft = false;
        boolean pickHang = false;
        boolean pickButter = false;

        telemetry.clearAll();
        while(!gamepad1.cross) {
            telemetry.addLine("Which Side");
            telemetry.addLine("right: "+ pickRight);
            telemetry.addLine("left: "+ pickLeft);
            if(gamepad1.square && !gamepad1.circle){
                pickRight = false;
                pickLeft = true;
            }
            if(!gamepad1.square && gamepad1.circle){
                pickRight = true;
                pickLeft = false;
            }
            telemetry.update();
        }
        if(pickRight){
            telemetry.clearAll();
            while(!gamepad1.right_bumper){
                telemetry.addLine("square-hang: "+ pickHang);
                telemetry.addLine("triangle-butter"+ pickButter);
                if(gamepad1.square && !gamepad1.triangle && !gamepad1.circle && !gamepad1.cross){
                    pickHang = true;
                }
                if(!gamepad1.square && gamepad1.triangle && !gamepad1.circle && !gamepad1.cross){
                    pickButter = true;
                }
                telemetry.update();
            }
        }

        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;

        TrajectoryActionBuilder startPosition = drive.actionBuilder(currentPose);


        TrajectoryActionBuilder choosenTrajectory;
        new SequentialAction(startPosition.build());
        choosenTrajectory = startPosition;
    }
}
