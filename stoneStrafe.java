package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "stoneStrafe", group = "autonomous")
public class stoneStrafe extends LinearOpMode {
    private DcMotor yeeter = null;
    private Servo armServo = null;
    private DcMotor armMotor = null;
    private Servo foundL = null;
    private Servo foundR = null;

    @Override
    public void runOpMode() {

        yeeter = hardwareMap.get(DcMotor.class, "yeeter");
        armServo = hardwareMap.get(Servo.class, "servo_arm");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        foundL = hardwareMap.get(Servo.class, "fl");
        foundR = hardwareMap.get(Servo.class, "fr");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        yeeter.setDirection(DcMotorSimple.Direction.REVERSE);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);;

        waitForStart();
        foundationUP();
        int currLiftPos = armMotor.getCurrentPosition();
        dropBlock();
        drive.setPoseEstimate(new Pose2d(-38,  -64, 0));

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-38,  -64, 0))
                .forward(28)
                .build();
        //grabBlock();
        drive.followTrajectory(traj);

        Trajectory traj1 = drive.trajectoryBuilder(traj.end()).back(7.0).build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end()).strafeLeft(85)
                .build();

        drive.followTrajectory(traj2);

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end()).forward(10.0).build();

        drive.followTrajectory(traj3);




        Trajectory traj5 = drive.trajectoryBuilder(traj3.end()).back(12).build();

        drive.followTrajectory(traj5);

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end()).strafeRight(95).build();

        drive.followTrajectory(traj6);



        Trajectory traj7 = drive.trajectoryBuilder(traj6.end()).forward(10.0).build();

        drive.followTrajectory(traj7);

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end()).back(10.0).build();

        drive.followTrajectory(traj8);


        Trajectory traj9 = drive.trajectoryBuilder(traj8.end()).strafeLeft(95)
                .build();

        drive.followTrajectory(traj9);

        Trajectory traj10 = drive.trajectoryBuilder(traj9.end()).forward(10)
                .build();

        drive.followTrajectory(traj9);


        Trajectory traj11 = drive.trajectoryBuilder(traj10.end()).lineTo(new Vector2d(-30, -10)).build();

        drive.followTrajectory(traj11);

        yeeter.setPower(1);
        sleep(2000);
        yeeter.setPower(0);


        sleep(30000);

    }
    public void grabBlock()
    {
        armServo.setPosition(.58);
        sleep(400);
    }
    public void dropBlock()
    {
        armServo.setPosition(.3);
        sleep(400);
    }
    public  void foundationUP()
    {
        foundL.setPosition(.6);
        foundR.setPosition(.2);
        sleep(400);
    }
    public  void foundationDown()
    {
        foundL.setPosition(.13);
        foundR.setPosition(.6);
        sleep(400);
    }
}