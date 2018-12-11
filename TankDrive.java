package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Locale;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * Created by team10382 on 9/14/2018.
 */

/*Log for "TankDrive"
    FTC - TecTigers 10382
    TeleOP
    Code made to test the tank drive of a code with gyroscope value
    Edit log: 
    A. Reyes on 11/17/18
       Added Log
       disabled code
    J. Bernstein & G. Koroda on 11/19/2018
        Implemented encoder code
        implemented OfficialTele into TankDrive
    A. Reyes on 12/7/18
        disabled code
*/
@TeleOp(name = "Tank Drive (Has gyroscope)", group = "Data Collection")
@Disabled
public class TankDrive extends OpMode {

     private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, rightLowerArm, leftLowerArm, upperArmMotor;
    
    CRServo leftIntakeServo, rightIntakeServo, hangingServo;
    BNO055IMU imu;
     Orientation angles;
    Acceleration gravity;
    
    


    @Override
    public void init(){
        telemetry.addData("Program", "Activated");
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        
         imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        composeTelemetry();
        
        
        
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        
        
        
        

         //initializes motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class,"right_back_drive");
        rightLowerArm = hardwareMap.get(DcMotor.class,"right_lower_arm");
        leftLowerArm = hardwareMap.get(DcMotor.class, "left_lower_arm");
        //upperArmMotor = hardwareMap.get(DcMotor.class,"upper_arm_drive");
       
        //iniializes Servos
        rightIntakeServo = hardwareMap.get(CRServo.class,"right_intake_servo"); // <-- Coninuous servo
        leftIntakeServo = hardwareMap.get(CRServo.class,"left_intake_servo"); // <-- Coninuous servo
        
        //sets direction of motors
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
       
        rightLowerArm.setDirection(DcMotor.Direction.REVERSE);
        leftLowerArm.setDirection(DcMotor.Direction.FORWARD);
        //upperArmMotor.setDirection(DcMotor.Direction.FORWARD);
        
        //set initial speeds of motor
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    
        rightLowerArm.setPower(0.0);
        leftLowerArm.setPower(0.0);
        //upperArmMotor.setPower(0.0);
        
        //set inital speeds of all Continuous Rotation servos
        leftIntakeServo.setPower(0.02); // 0.02 is not moving for continuous servo
        rightIntakeServo.setPower(0.02);
        hangingServo.setPower(0.02);
    }

    @Override
    public void init_loop() {
    }
 void composeTelemetry() {

        
        telemetry.addAction(new Runnable() { @Override public void run()
                {
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
                }
            });

        telemetry.addLine()
            .addData("status", new Func<String>() {
                @Override public String value() {
                    return imu.getSystemStatus().toShortString();
                    }
                })
            .addData("calib", new Func<String>() {
                @Override public String value() {
                    return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
            .addData("heading", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
            .addData("roll", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
            .addData("pitch", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
            .addData("grvty", new Func<String>() {
                @Override public String value() {
                    return gravity.toString();
                    }
                })
            .addData("mag", new Func<String>() {
                @Override public String value() {
                    return String.format(Locale.getDefault(), "%.3f",
                            Math.sqrt(gravity.xAccel*gravity.xAccel
                                    + gravity.yAccel*gravity.yAccel
                                    + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    @Override
    public void loop(){
        
        //driving using tank drive
        leftFrontDrive.setPower(gamepad1.left_stick_y);
        leftBackDrive.setPower(gamepad1.left_stick_y);
        
        rightFrontDrive.setPower(gamepad1.right_stick_y);
        rightBackDrive.setPower(gamepad1.right_stick_y);
        
        //Raise and lower arm
        leftLowerArm.setPower(gamepad2.left_stick_y);
        rightLowerArm.setPower(gamepad2.left_stick_y);
        
        //raise and lower intake
        //upperArmMotor.setPower(gamepad2.right_stick_y);
        
        //Intake mechanism
        if(gamepad2.right_trigger == 1){
            leftIntakeServo.setPower(-1.0);
            rightIntakeServo.setPower(1.0);
        }else if(gamepad2.left_trigger == 1.0){
            leftIntakeServo.setPower(1.0);
            rightIntakeServo.setPower(-1.0);
        }else{
            leftIntakeServo.setPower(0.02);
            rightIntakeServo.setPower(0.02);
        }
        
        //hanging manipulator
        if(gamepad2.x){
            hangingServo.setPower(1.0);
        }else if(gamepad2.y){
            hangingServo.setPower(-1.0);
        }else{
            hangingServo.setPower(0.02);
        }
        
        
        telemetry.addData("gamepad2.x - ",gamepad2.x);
        telemetry.addData("gamepad2.y - ",gamepad2.y);
        telemetry.addData("gamepad2.right_trigger - ", gamepad2.right_trigger);
        telemetry.addData("gamepad2.left_trigger - ", gamepad2.left_trigger);
        telemetry.addData("rightIntakeServo", rightIntakeServo.getPower());
        telemetry.addData("leftIntakeServo", leftIntakeServo.getPower());
        telemetry.addData("hangingServo", hangingServo.getPower());
        telemetry.addData("RightLowerArmValue:" , rightLowerArm.getCurrentPosition());
       // telemetry.addData("LeftLowerArmValue:" , leftLowerArm.getCurrentPostion());
       // telemtery.addData("UpperArmValue:" , upperArmMotor.getCurrentPosition());
        

    }
}
