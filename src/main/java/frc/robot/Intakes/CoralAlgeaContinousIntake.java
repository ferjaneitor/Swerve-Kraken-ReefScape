package frc.robot.Intakes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.Intakes.Algae.AlgaeSubSystem;
import frc.robot.Intakes.Coral.CoralSubSystem;

/**
 * CoralAlgeaContinousIntake es un comando continuo que activa simultáneamente
 * la succión (intake) de los subsistemas Algea y Coral. Permite controlar la dirección
 * (invertida o no) de ambos intakes mediante la variable isInverted.
 *
 * Descripción:
 * - En initialize(), se calcula la velocidad final a partir de constants.CoralAlgaeFinalVelocity
 *   y la bandera isInverted.
 * - En execute(), se activa el intake de Algea con finalVelocity y el de Coral con -finalVelocity,
 *   permitiendo que ambos subsistemas trabajen de forma conjunta (posiblemente en sentidos opuestos).
 * - En end(), se detienen ambos intakes.
 * - El método isFinished() retorna siempre false, lo que significa que este comando no concluye por sí solo.
 *
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.1
 */
public class CoralAlgeaContinousIntake extends Command {

    /**
     * Indica si se invierte la dirección de los intakes.
     */
    private boolean isInverted;

    /**
     * Subsistema Algea que controla uno de los mecanismos de intake.
     */
    private AlgaeSubSystem algeaSubSystem;

    /**
     * Subsistema Coral que controla el otro mecanismo de intake.
     */
    private CoralSubSystem coralSubSystem;

    /**
     * Velocidad final que se aplicará a los motores, calculada con base en
     * constants.CoralAlgaeFinalVelocity y la bandera isInverted.
     */
    private double finalVelocity;

    /**
     * Construye un comando continuo para accionar de forma simultánea los intakes de Algea y Coral.
     *
     * @param isInverted       true para invertir la dirección de los intakes.
     * @param algeaSubSystem   Subsistema de Algea que maneja sus motores de intake.
     * @param coralSubSystem   Subsistema de Coral que maneja sus motores de intake.
     */
    public CoralAlgeaContinousIntake(boolean isInverted, AlgaeSubSystem algeaSubSystem, CoralSubSystem coralSubSystem) {
        this.isInverted = isInverted;
        this.algeaSubSystem = algeaSubSystem;
        this.coralSubSystem = coralSubSystem;
    }

    /**
     * Inicializa el comando, calculando la velocidad final según constants.CoralAlgaeFinalVelocity
     * y la inversión de dirección.
     */
    @Override
    public void initialize() {
        finalVelocity = constants.CoralAlgaeFinalVelocity * (isInverted ? -1 : 1);
    }

    /**
     * Se llama repetidamente mientras el comando está activo. Activa el subsistema Algea
     * con finalVelocity y el subsistema Coral con -finalVelocity.
     */
    @Override
    public void execute() {
        algeaSubSystem.enableAlgaeIntake(finalVelocity);
        coralSubSystem.enableCoralIntake(finalVelocity);
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
     * Este comando nunca finaliza por sí solo; retorna siempre false.
     *
     * @return false.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
