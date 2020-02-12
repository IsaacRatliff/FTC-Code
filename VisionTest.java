package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Vision Test", group="Linear OpMode")

public class VisionTest extends VuforiaOp{
    @Override
    public void runOpMode(){
        ElapsedTime runtime = new ElapsedTime();
        initVision(hardwareMap);
        initialize(hardwareMap, telemetry);
        activateVision();
        telemetry.addData("Status", "waiting");
        waitForStart();
        runtime.reset();
        while(opModeIsActive()){
            calcOffset();
            if(raw_offset == null){
                telemetry.addData("Reading", "(%.2f S) null", runtime.seconds());
                strafeLeftSpd(0.1);
            }
            else{
                telemetry.addData("Reading", "(%.2f S) x: %.0f, y: %.0f, z: %.0f", runtime.seconds(), red_off, blue_off, green_off);
                strafeRightSpd(red_off/1000);
            }
            telemetry.update();
        }
    }
}
