package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorConstants {

    public enum Heights {

        TOP(1.7),
        MIDDLE(0.7),
        BOTTOM(0);

        public final double height;

        Heights(double height) {
            this.height = height;
        }
    }

    public static final TalonFX MOTOR = new TalonFX(20);

   public static final CANcoder ENCODER = new CANcoder(21);

   public static final DigitalInput LIMIT_SWITCH = new DigitalInput(30);

}
