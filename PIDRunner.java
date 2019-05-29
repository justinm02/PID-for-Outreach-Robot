package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "PIDTest", group = "Autonomous")
public class PIDRunner extends LinearOpMode {
    private ElapsedTime runtime;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx[] motors = {leftFront, leftBack, rightFront, rightBack};
    private BNO055IMU imu;
    private PID pid = new PID(.01, 0.0003, .0013);

    public void runOpMode() {
        initialize();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        move(0, .25);
    }

    public void initialize() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        motors = new DcMotorEx[] {leftFront, leftBack, rightFront, rightBack};

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        runtime = new ElapsedTime();
    }

    public void move(double targetHeading, double power) {
        reset();
        setTargetPosition(200000);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
            correction(currentAngle() - targetHeading, power, targetHeading);
        }

        halt();
    }

    public double currentAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void reset() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void setTargetPosition(int targetPosition) {
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(targetPosition);
        }
    }

    public void halt() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

    public void correction(double error, double power, double targetHeading)
    {
        /*if (error > 0) {
            leftFront.setPower(power + pid.getCorrection(currentAngle() - targetHeading));
            rightFront.setPower(power);
            leftBack.setPower(power + pid.getCorrection(currentAngle() - targetHeading));
            rightBack.setPower(power);
        }
        else {
            leftFront.setPower(power);
            rightFront.setPower(power + pid.getCorrection(currentAngle() - targetHeading));
            leftBack.setPower(power);
            rightBack.setPower(power + pid.getCorrection(currentAngle() - targetHeading));
        }*/
        leftFront.setPower(power + pid.getCorrection(currentAngle() - targetHeading, runtime));
        rightFront.setPower(power - pid.getCorrection(currentAngle() - targetHeading, runtime));
        leftBack.setPower(power + pid.getCorrection(currentAngle() - targetHeading, runtime));
        rightBack.setPower(power - pid.getCorrection(currentAngle() - targetHeading, runtime));

        telemetry.addData("leftFront Power", leftFront.getPower());
        telemetry.addData("rightFront Power", rightFront.getPower());
        telemetry.addData("leftBack Power", leftBack.getPower());
        telemetry.addData("rightBack Power", rightBack.getPower());
        telemetry.addData("Current angle", currentAngle());
        telemetry.update();

    }

}
