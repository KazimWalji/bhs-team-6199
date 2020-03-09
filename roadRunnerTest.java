package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import kotlin.Unit;


@Autonomous(name = "roadRunnerTest", group = "autonomous")
public class roadRunnerTest extends LinearOpMode {
    private DcMotor yeeter = null;
    private Servo armServo = null;
    private DcMotor armMotor = null;
    private Servo foundL = null;
    private Servo foundR = null;
    boolean isBusy = false;

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
        double armPower = 0;

        waitForStart();
        foundationUP();
        int currLiftPos = armMotor.getCurrentPosition();
        dropBlock();
        drive.setPoseEstimate(new Pose2d(-38,  -64, 0));

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-38,  -64, 0))
                .forward(30)
                .build();
        drive.followTrajectory(traj);

        grabBlock();

        Trajectory traj1 = drive.trajectoryBuilder(traj.end()).back(8.0).build();

        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(90));
        telemetry.addData("done", "done");
        telemetry.update();
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(traj1.end().getX(), traj1.end().getY(), Math.toRadians(90))).forward(85).addDisplacementMarker(2.5, () -> {
            trueIsBusy();
        })
                .build();
        drive.followTrajectory(traj2);

        drive.turn(Math.toRadians(-90));

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(traj2.end().getX(), traj2.end().getY(), Math.toRadians(0))).forward(10.0).build();

        drive.followTrajectory(traj3);

        armServo.setPosition(.3);

        foundationDown();


        Trajectory traj4 = drive.trajectoryBuilder(traj3.end()).lineTo(new Vector2d(-30, -10)).build();

        drive.followTrajectory(traj4);

        drive.turn(Math.toRadians(90));

        foundationUP();

        Trajectory traj6 = drive.trajectoryBuilder(traj4.end()).back(63).build();

        drive.followTrajectory(traj6);

        drive.turn(Math.toRadians(-90));

        Trajectory traj7 = drive.trajectoryBuilder(new Pose2d(traj6.end().getX(),traj6.end().getY(), Math.toRadians(0))).forward(10.0).build();

        drive.followTrajectory(traj7);

        grabBlock();

        Trajectory traj9 = drive.trajectoryBuilder(traj7.end()).back(10.0).build();

        drive.followTrajectory(traj9);

        drive.turn(Math.toRadians(90));

        Trajectory traj8 = drive.trajectoryBuilder(new Pose2d(traj9.end().getX(), traj9.end().getY(), Math.toRadians(90))).forward(100.0).addSpatialMarker(new Vector2d(10, -40), () -> {
            trueIsBusy();
        }).build();

        drive.followTrajectory(traj8);

        dropBlock();

        drive.turn(Math.toRadians(180));

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

    }
    public void trueIsBusy()
    {
        isBusy = true;
    }
}