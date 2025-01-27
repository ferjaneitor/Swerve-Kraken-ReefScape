package frc.robot.Intakes.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeConstants;

/**
 * @AlgaeEnableIntakeCmd es un comando continuo que habilita la succión (intake)
 * del subsistema Algea a una velocidad fija. El sentido de giro se determina
 * según la bandera {@code isInverted}, pudiendo invertirse para distintas 
 * necesidades (por ejemplo, tomar o expulsar piezas).
 *
 * <ul>
 *   <li>En {@code initialize()}, se calcula {@code finalVelocity} basándose en
 *       {@link AlgaeConstants#intakeVelocity} y la inversión según {@code isInverted}.</li>
 *   <li>En {@code execute()}, se aplica esa velocidad a los motores del subsistema
 *       para mantener el intake funcionando.</li>
 *   <li>En {@code end()}, se detienen los motores (por ejemplo, cuando se interrumpe
 *       el comando o finaliza el periodo de operación).</li>
 *   <li>{@code isFinished()} siempre retorna {@code false}, indicando que el
 *       comando no concluye automáticamente.</li>
 * </ul>
 *
 * @Autor:  Fernando Joel Cruz Briones
 * @Versión: 1.0
 */
public class AlgaeEnableIntakeCmd extends Command {

    /**
     * Bandera que determina si se invierte la dirección de los motores.
     */
    private boolean isInverted;

    /**
     * Referencia al subsistema Algea, que maneja el mecanismo de intake.
     */
    private AlgeaSubSystem algeaSubSystem;

    /**
     * Velocidad final calculada según la constante {@code intakeVelocity}
     * y el estado de inversión.
     */
    private double finalVelocity;

    /**
     * Construye un nuevo comando para habilitar el intake de Algea.
     *
     * @param isInverted     Indica si se invierte el sentido del motor (true/false).
     * @param algeaSubSystem Instancia del subsistema Algea a controlar.
     */
    public AlgaeEnableIntakeCmd(boolean isInverted, AlgeaSubSystem algeaSubSystem) {
        this.algeaSubSystem = algeaSubSystem;
        this.isInverted = isInverted;
    }

    /**
     * Inicializa el comando, calculando la velocidad final según
     * {@link AlgaeConstants#intakeVelocity} y la inversión.
     */
    @Override
    public void initialize() {
        finalVelocity = AlgaeConstants.intakeVelocity * (isInverted ? -1 : 1);
    }

    /**
     * Se llama repetidamente mientras el comando está activo.
     * Ajusta la velocidad del subsistema Algea al valor {@code finalVelocity}.
     */
    @Override
    public void execute() {
        algeaSubSystem.enableAlgaeIntake(finalVelocity);
    }

    /**
     * Se llama cuando el comando termina o es interrumpido, deteniendo
     * los motores del intake.
     *
     * @param interrupted Indica si el comando fue interrumpido.
     */
    @Override
    public void end(boolean interrupted) {
        algeaSubSystem.stopAlgaeIntake();
    }

    /**
     * Devuelve {@code false} para indicar que este comando no concluye
     * por sí solo. Normalmente se interrumpe desde otro lugar o por
     * un cambio de modo del robot.
     *
     * @return false siempre.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
