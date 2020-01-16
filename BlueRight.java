
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "BlueRight", group="Sky autonomous")
//@Disabled//comment out this line before using
public class BlueRight extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private ElapsedTime runTime = new ElapsedTime();
    private DcMotor armMotor = null;
    private Servo armServo = null;
    BNO055IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // make sure the gyro is calibrated before continuing


        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armServo = hardwareMap.get(Servo.class, "servo_arm");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int currLiftPos = armMotor.getCurrentPosition();
        int[] pos = {currLiftPos, 175, 400, 1100, 1899, 2879, 3099, 1299, 1499, 1699};

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        armServo.setPosition(.5);
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("pos","right");
            telemetry.update();

            encoder(1100, 1100, 1100, 1100, .4, 5);
            encoder(700, -700, -700, 700, .4, 3);
            encoder(100, 100, 100, 100, .25, 3);
            armServo.setPosition(.8);
            sleep(1000);
            encoder(-300, -300, -300, -300, .3, 3);

            turnAngle(85, 93, -.3, -.3, .3, .3);
            encoder(2500, 2500, 2500, 2500, .4, 7);
            armServo.setPosition(.5);
            sleep(1000);

            encoder(-2900, -2900, -2900, -2900, .4, 7);
            turnAngle(-3, 3, .3, .3, -.3, -.3);


            encoder(600, 600, 600, 600, .3, 3);
            encoder(-600, -600, -600, -600, .3, 3);


            encoder(500, -500, -500, 500, .3, 3);
            encoder(700, 700, 700, 700, .1, 3);
            armServo.setPosition(.8);
            sleep(1000);
            encoder(-600, -600, -600, -600, .3, 3);

            turnAngle(85, 93, -.3, -.3, .3, .3);
            encoder(3400, 3400, 3400, 3400, .8, 7);
            armServo.setPosition(.5);
            sleep(1000);
            encoder(-400, -400, -400, -400, .4, 5);
            armMotor.setTargetPosition(pos[0]);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(.4);
            while (armMotor.isBusy() && opModeIsActive())
            {

            }
            telemetry.addData("pos", armMotor.getCurrentPosition());
            telemetry.update();
            armMotor.setPower(0);
            sleep (10000);

        }
        
    }

        public void encoder (int lf, int rf, int lr, int rr, double pow, int sec )
        {
            pow = pow+.1;
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

            rightFront.setTargetPosition(rf);
            leftRear.setTargetPosition(lr);
            rightRear.setTargetPosition(rr);
            leftFront.setTargetPosition(lf);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode((DcMotor.RunMode.RUN_TO_POSITION));


            leftFront.setPower(pow);
            rightFront.setPower(pow);
            rightRear.setPower(pow);
            leftRear.setPower(pow);
            leftFront.setPower(pow);
            runTime.reset();
            while (rightRear.isBusy() || leftRear.isBusy() || rightFront.isBusy() | leftFront.isBusy() && opModeIsActive() && runTime.seconds() < sec) {


            }

            leftFront.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            leftRear.setPower(0);
        }

        public void turnAngle (int firstA, int secA, double pow1, double pow2, double pow3, double pow4)
        {
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double currAngle;
            Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currAngle = (angle.firstAngle+360) % 360;
            telemetry.addData("currAngle", currAngle);
            telemetry.update();
            while ((currAngle < firstA|| currAngle > secA) && opModeIsActive()) {
                leftFront.setPower(pow1);
                leftRear.setPower(pow2);
                rightRear.setPower(pow3);
                rightFront.setPower(pow4);
                angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currAngle = (angle.firstAngle+360) % 360;
                telemetry.addData("currAngle", currAngle);
                telemetry.update();
            }
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
            rightFront.setPower(0);

        }
    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle+360)%360;
    }


}
