package frc.robot.Intakes.Algae;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeConstants;

public class AlgaeEnableIndividualIntake extends Command {
    
    /**
     * Bandera que determina si se invierte la dirección de los motores.
     */
    Supplier<Double> Direction;
    
    private boolean isRight;
    private boolean isInverted;

    /**
     * Referencia al subsistema Algea, que maneja el mecanismo de intake.
     */
    private AlgaeSubSystem algeaSubSystem;

    /**
     * Velocidad final calculada según la constante intakeVelocity y el estado de inversión.
     */
    private double finalVelocity;

    private SparkMax SelectedMotor;

    /**
     * Construye un nuevo comando para habilitar el intake de Algea.
     *
     * @param invertDirection  Indica si se invierte el sentido del motor (true/false).
     * @param algeaSubSystem   Instancia del subsistema Algea a controlar.
     */
    public AlgaeEnableIndividualIntake(boolean IsInverted, boolean IsRight, AlgaeSubSystem algeaSubSystem, Supplier<Double> Direction) {
        this.algeaSubSystem = algeaSubSystem;
        this.isInverted = IsInverted;
        this.Direction = Direction;
        this.isRight = IsRight;
        addRequirements(algeaSubSystem);
        this.SelectedMotor = algeaSubSystem.getSparkMax(IsRight);
    
    }

    /**
     * Inicializa el comando, calculando la velocidad final según
     * AlgaeConstants.intakeVelocity y la inversión.
     */
    @Override
    public void initialize() {
        finalVelocity = AlgaeConstants.intakeVelocity * (Direction.get() > 0.5 ? Direction.get() : 0);
    }

    /**
     * Se llama repetidamente mientras el comando está activo.
     * Ajusta la velocidad del subsistema Algea al valor de finalVelocity.
     */
    @Override
    public void execute() {
        SelectedMotor.set(finalVelocity);
    }

    /**
     * Se llama cuando el comando termina o es interrumpido, deteniendo
     * los motores del intake.
     *
     * @param interrupted Indica si el comando fue interrumpido.
     */
    @Override
    public void end(boolean interrupted) {
        SelectedMotor.set(0);
    }

    /**
     * Devuelve false para indicar que este comando no concluye por sí solo.
     *
     * @return false siempre.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
    
}
