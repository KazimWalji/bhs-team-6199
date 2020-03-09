
package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name= "TeleOP1 controller", group="Linear Opmode")
//@Disabled
public class TeleOP1Controller extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private Servo armServo = null;
    private Servo foundL = null;
    private Servo foundR = null;
    private DcMotor armMotor = null;
    private DcMotor yeeter = null;
    private Servo capstone = null;
    private DistanceSensor sensorRange;
    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear  = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        yeeter = hardwareMap.get(DcMotor.class, "yeeter");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        yeeter.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        armServo = hardwareMap.get(Servo.class, "servo_arm");
        capstone = hardwareMap.get(Servo.class, "cap");
        foundL = hardwareMap.get(Servo.class, "fl");
        foundR = hardwareMap.get(Servo.class, "fr");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean armM = false;
        //Wait for the game to start (driver presses PLAY)
        waitForStart();
        foundationUP();
        telemetry.addData("cap:", capstone.getPosition());
        telemetry.update();
        int currLiftPos = armMotor.getCurrentPosition();
        int[] pos = {currLiftPos, 400, 1100, 1800, 2500, 3200, 3800, 1499, 1699};
        armServo.setPosition(.1);
        capstone.setPosition(.35);
        double x = 1.2;
        Double c = capstone.getPosition();
        //armServo.setPosition(0);
        runtime.reset();
        // Look for DPAD presses to change the selection



        boolean buttonPrev = false;
        while (opModeIsActive()) {

            if (gamepad1.dpad_up)
            {

                capstone.setPosition(0.07);

            }
            if (gamepad2.dpad_down)
            {

                capstone.setPosition(.33);

            }
            if(gamepad1.left_bumper)
            {
                foundationDown();
            }
            if(gamepad1.right_bumper)
            {
                foundationUP();
            }
            if(gamepad1.right_trigger!=0){
                yeeter.setPower(gamepad1.left_trigger);
            }else{
                yeeter.setPower(0);
            }
            if(gamepad1.right_stick_y == 0)
            {

                armMotor.setPower(0);
            }
            if(armMotor.getCurrentPosition() < (pos[0] + 10) && -gamepad1.right_stick_y < 0)
            {
                armMotor.setPower(0);
            }
            else
            {
                armMotor.setPower(-gamepad1.right_stick_y);
            }
            if (gamepad1.x)
            {
                telemetry.addData("boolean", buttonPrev);
                telemetry.addData("pos", armServo.getPosition());
                telemetry.update();
            }
            if(gamepad1.x){
                armServo.setPosition(.3);
            }
            if (gamepad1.a)
            {
                armServo.setPosition(.56);
            }
            if (gamepad1.left_trigger != 0)
            {
                x = .7;
                if (sensorRange.getDistance(DistanceUnit.INCH) < 2.0 )
                    armServo.setPosition((.58));
            }
            else
            {
                x = 1.2;
            }

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = (-gamepad1.right_stick_x);
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            leftFront.setPower(v1 * x);
            rightFront.setPower(v2* x);
            leftRear.setPower(v3* x);
            rightRear.setPower(v4* x);

        }
    }
    public  void foundationUP()
    {
        foundL.setPosition(.6);
        foundR.setPosition(.2);
    }
    public  void foundationDown()
    {
        foundL.setPosition(.15);
        foundR.setPosition(.63);
    }

}