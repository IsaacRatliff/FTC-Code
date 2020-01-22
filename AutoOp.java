package workspace_;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AutoOp extends LinearOpMode {
    private DcMotor leftfront = null;
    private DcMotor leftback = null;
    private DcMotor rightfront = null;
    private DcMotor rightback = null;
    protected DcMotor arm = null;
    protected DcMotor modelX = null;
    protected Servo finger1 = null;
    protected Servo finger2 = null;
    protected Servo claw = null;
    
    private ElapsedTime runtime = new ElapsedTime();
    private Telemetry telemetry_ = null;
    
    private double speed = 0.55*0.7/2;
    private double turn90 = 0.85/2;
    private double armSpd = 0.5;
    protected double wheelPower = 0.5;
    
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, boolean has_arm, boolean has_claw){
        leftfront = hardwareMap.get(DcMotor.class, "leftDrive_0");
        leftback = hardwareMap.get(DcMotor.class, "leftDrive_1");
        rightfront = hardwareMap.get(DcMotor.class, "rightDrive_0");
        rightback = hardwareMap.get(DcMotor.class, "rightDrive_1");
        modelX = hardwareMap.get(DcMotor.class, "ModelX");
        finger1 = hardwareMap.get(Servo.class, "finger1");
        finger2 = hardwareMap.get(Servo.class, "finger2");
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);
        modelX.setDirection(DcMotor.Direction.FORWARD);
        telemetry_ = telemetry;
        if(has_arm){
            arm = hardwareMap.get(DcMotor.class, "arm");
            arm.setDirection(DcMotor.Direction.FORWARD);
        }
        if(has_claw){
            claw = hardwareMap.get(Servo.class, "claw");
        }
    }
    
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, boolean has_arm){
        initialize(hardwareMap, telemetry, has_arm, has_arm);
    }
    
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        initialize(hardwareMap, telemetry, true, true);
    }
    
    public void leftWheels(double spd){
        leftfront.setPower(spd);
        leftback.setPower(spd);
    }
    
    public void rightWheels(double spd){
        rightfront.setPower(spd);
        rightback.setPower(spd);
    }
    
    public void driveSpd(double spd){
        leftWheels(spd);
        rightWheels(spd);
    }
    
    public void turnRightSpd(double spd){
        leftWheels(spd);
        rightWheels(-spd);
    }
    
    public void turnLeftSpd(double spd){
        leftWheels(-spd);
        rightWheels(spd);
    }
    
    public void strafeLeftSpd(double spd){
        leftfront.setPower(-spd);
        leftback.setPower(spd);
        rightfront.setPower(spd);
        rightback.setPower(-spd);
    }
    
    public void strafeRightSpd(double spd){
        leftfront.setPower(spd);
        leftback.setPower(-spd);
        rightfront.setPower(-spd);
        rightback.setPower(spd);
    }
    
    public void forward(double tiles){
        leftWheels(wheelPower);
        rightWheels(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < tiles * speed / wheelPower)){
            telemetry_.addData("Path", "Forward: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        stopWheels();
    }
    public void backward(double tiles){
        leftWheels(-wheelPower);
        rightWheels(-wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < tiles * speed / wheelPower)){
            telemetry_.addData("Path", "Backward: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        stopWheels();
    }
    public void left(double quarters){
        leftWheels(-wheelPower);
        rightWheels(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < quarters * turn90 / wheelPower)){
            telemetry_.addData("Path", "Turning Right: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        stopWheels();
    }
    public void right(double quarters){
        leftWheels(wheelPower);
        rightWheels(-wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < quarters * turn90 / wheelPower)){
            telemetry_.addData("Path", "Turning Right: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        stopWheels();
    }
    public void armUp(double pct){
        arm.setPower(0.4);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < pct * armSpd)){
            telemetry_.addData("Path", "Raising arm: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        arm.setPower(0.15);
    }
    public void armUp(){
        armUp(1.0);
    }
    public void armDown(double pct){
        arm.setPower(-0.1);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < pct * armSpd)){
            telemetry_.addData("Path", "Lowering arm: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        arm.setPower(0.15);
    }
    public void armDown(){
        armDown(1.0);
    }
    public void fingerUp(){
        finger1.setPosition(1.0);
        finger2.setPosition(0.0);
    }
    public void fingerDown(){
        finger1.setPosition(0.0);
        finger2.setPosition(1.0);
    }
    public void openClaw(){
        claw.setPosition(0.0);
    }
    public void closeClaw(){
        claw.setPosition(1.0);
    }
    public void modelXUp(){
        modelX.setPower(0.5);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 0.1)){
            telemetry_.addData("Path", "Raising ModelX: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        arm.setPower(0.0);
    }
    public void modelXDown(){
        modelX.setPower(-0.5);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 0.1)){
            telemetry_.addData("Path", "Lowering ModelX: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        arm.setPower(-0.1);
    }
    public void pause(double seconds){
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < seconds)){
            telemetry_.addData("Path", "Wating: %2.5f / %2.5f S", runtime.seconds(), seconds);
            telemetry_.update();
        }
    }
    @Deprecated
    public void stopMotion(){
        stopWheels();
    }
    public void stopWheels(){
        leftWheels(0.0);
        rightWheels(0.0);
    }
}
