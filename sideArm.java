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


@Autonomous(name = "sideArm", group = "autonomous")
public class sideArm extends LinearOpMode {
    private DcMotor yeeter = null;
    private DcMotor armMotor = null;
    private Servo foundL = null;
    private Servo foundR = null;
    boolean isBusy = false;

    @Override
    public void runOpMode() {

        yeeter = hardwareMap.get(DcMotor.class, "yeeter");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        foundL = hardwareMap.get(Servo.class, "fl");
        foundR = hardwareMap.get(Servo.class, "fr");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        yeeter.setDirection(DcMotorSimple.Direction.REVERSE);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);;
        double armPower = 0;

        waitForStart();
        foundationUP();
        int currLiftPos = armMotor.getCurrentPosition();
        dropBlock();
        drive.setPoseEstimate(new Pose2d(38,  64, 0));

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(38,  64, 0))
                .forward(25)
                .build();
        drive.followTrajectory(traj);
        drive.turn(Math.toRadians(90));
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(traj.end().getX(), traj.end().getY(), Math.toRadians(90)))
                .strafeRight(5)
                .build();
        drive.followTrajectory(traj1);
        sleep(500);
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(50, 35))
                .build();
        drive.followTrajectory(traj2);

        sleep(500);

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
               .lineTo(new Vector2d(-58, 32))
                .build();
        drive.followTrajectory(traj3);
        sleep(500);
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(0, 35))
                .build();
        drive.followTrajectory(traj4);

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineTo(new Vector2d(-58, 38))
                .build();
        drive.followTrajectory(traj5);

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineTo(new Vector2d(0, 35))
                .build();
        drive.followTrajectory(traj6);

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineTo(new Vector2d(58, 32))
                .build();

        drive.followTrajectory(traj7);

        drive.turn(Math.toRadians(-90));

        foundationDown();

        Trajectory traj8 = drive.trajectoryBuilder(new Pose2d(traj7.end().getX(), traj7.end().getY(), Math.toRadians(0)))
                .lineTo(new Vector2d(28, 48))
                .build();
        drive.followTrajectory(traj8);

        drive.turn(Math.toRadians(90));


        yeeter.setPower(1);
        sleep(2000);
        yeeter.setPower(0);


        sleep(30000);

    }
    public void grabBlock()
    {
        //armServo.setPosition(.58);
        sleep(400);
    }
    public void dropBlock()
    {
        //armServo.setPosition(.3);
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

    }
    public void trueIsBusy()
    {
        isBusy = true;
    }
}