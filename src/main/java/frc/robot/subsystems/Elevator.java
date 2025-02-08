package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
public class Elevator extends SubsystemBase {
    
    private TalonFX leader;
    private TalonFX follower;
     
    private static final int LEADER_ID = 14;
    private static final int FOLLOWER_ID = 15;

    public Elevator() {
        this.leader = new TalonFX(LEADER_ID, "Canivore");
        this.follower = new TalonFX(FOLLOWER_ID, "Canivore");
        follower.setControl(new Follower(leader.getDeviceID(), false));
    }

    


}
