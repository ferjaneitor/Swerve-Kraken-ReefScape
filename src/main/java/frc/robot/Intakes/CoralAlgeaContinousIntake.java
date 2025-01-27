package frc.robot.Intakes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.Intakes.Algae.AlgeaSubSystem;
import frc.robot.Intakes.Coral.CoralSubSystem;

/**
 * @{@code CoralAlgeaContinousIntake} es un comando continuo que activa
 * simultáneamente la succión (intake) de los subsistemas Algea y Coral.
 * Permite controlar la dirección (invertida o no) de ambos intakes mediante
 * la variable {@code isInverted}.
 *
 * <ul>
 *   <li>En {@code initialize()}, se calcula la velocidad final a partir de 
 *       {@link constants#CoralAlgaeFinalVelocity} y la bandera {@code isInverted}.</li>
 *   <li>En {@code execute()}, se activa el intake de Algea con 
 *       {@code finalVelocity} y el de Coral con {@code -finalVelocity}, 
 *       permitiendo que ambos subsistemas trabajen de forma conjunta 
 *       (posiblemente en sentidos opuestos).</li>
 *   <li>En {@code end()}, se detienen ambos intakes.</li>
 *   <li>{@code isFinished()} siempre retorna {@code false}, lo que significa 
 *       que este comando no concluye por sí solo y debe ser interrumpido.</li>
 * </ul>
 *
 * @Autor:  Fernando Joel Cruz Briones
 * @Versión: 1.0
 */
public class CoralAlgeaContinousIntake extends Command {

    /**
     * Indica si se invierte la dirección de los intakes.
     */
    private boolean isInverted;

    /**
     * Subsistema Algea que controla uno de los mecanismos de intake.
     */
    private AlgeaSubSystem algeaSubSystem;

    /**
     * Subsistema Coral que controla el otro mecanismo de intake.
     */
    private CoralSubSystem coralSubSystem;

    /**
     * Velocidad final que se aplicará a los motores, calculada con base
     * en {@link constants#CoralAlgaeFinalVelocity} y la bandera 
     * {@code isInverted}.
     */
    private double finalVelocity;

    /**
     * Construye un comando continuo para accionar de forma simultánea
     * los intakes de Algea y Coral.
     *
     * @param isInverted       {@code true} para invertir la dirección de los intakes.
     * @param algeaSubSystem   Subsistema de Algea que maneja sus motores de intake.
     * @param coralSubSystem   Subsistema de Coral que maneja sus motores de intake.
     */
    public CoralAlgeaContinousIntake(boolean isInverted, AlgeaSubSystem algeaSubSystem, CoralSubSystem coralSubSystem) {
        this.isInverted = isInverted;
        this.algeaSubSystem = algeaSubSystem;
        this.coralSubSystem = coralSubSystem;
    }

    /**
     * Inicializa el comando, calculando la velocidad final según 
     * {@link constants#CoralAlgaeFinalVelocity} y la inversión de dirección.
     */
    @Override
    public void initialize() {
        finalVelocity = constants.CoralAlgaeFinalVelocity * (isInverted ? -1 : 1);
    }

    /**
     * Se llama repetidamente mientras el comando está activo. Activa
     * el subsistema Algea con {@code finalVelocity} y el subsistema
     * Coral con la velocidad opuesta, {@code -finalVelocity}.
     */
    @Override
    public void execute() {
        algeaSubSystem.enableAlgaeIntake(finalVelocity);
        coralSubSystem.enableCoralIntake(-finalVelocity);
    }

    /**
     * Se llama al finalizar o interrumpir el comando, deteniendo ambos subsistemas.
     *
     * @param interrupted Indica si el comando terminó de forma normal o fue interrumpido.
     */
    @Override
    public void end(boolean interrupted) {
        algeaSubSystem.stopAlgaeIntake();
        coralSubSystem.stopCoralIntake();
    }

    /**
     * El comando nunca finaliza por sí solo; retorna siempre {@code false}.
     *
     * @return Siempre {@code false}.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
