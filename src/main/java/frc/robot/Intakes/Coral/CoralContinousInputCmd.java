package frc.robot.Intakes.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralConstants;

/**
 * CoralContinousInputCmd es un comando continuo que activa la succión (intake)
 * del subsistema Coral a una velocidad específica, con la opción de invertir
 * la dirección de succión.
 *
 * La velocidad de intake se obtiene de CoralConstants.MotorsIntakeVelocity y se
 * multiplica por -1 si invertDirection es verdadero.
 *
 * Al finalizar o ser interrumpido, se detiene el intake.
 *
 * El método isFinished() retorna siempre false, por lo que este comando no concluye
 * de forma automática.
 *
 * Se añaden los requisitos del subsistema CoralSubSystem para asegurar la exclusividad
 * de su control.
 *
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.1
 */
public class CoralContinousInputCmd extends Command {

    /**
     * Indica si se invierte el sentido de giro del intake.
     */
    private final boolean directionInverted;

    /**
     * Velocidad calculada para el intake, basada en CoralConstants.MotorsIntakeVelocity
     * y la bandera directionInverted.
     */
    private final double intakeVelocity;

    /**
     * Subsistema que controla el mecanismo Coral (intake).
     */
    private final CoralSubSystem coralSubSystem;

    /**
     * Constructor que configura si se invierte la dirección del intake y
     * a qué subsistema pertenece este comando.
     *
     * @param invertDirection  true para invertir la dirección del intake.
     * @param coralSubSystem   Subsistema de Coral responsable del intake.
     */
    public CoralContinousInputCmd(boolean invertDirection, CoralSubSystem coralSubSystem) {
        this.directionInverted = invertDirection;
        this.intakeVelocity = CoralConstants.MotorsIntakeVelocity * (directionInverted ? -1 : 1);
        this.coralSubSystem = coralSubSystem;
        addRequirements(coralSubSystem);
    }

    /**
     * Inicializa el comando. No se realiza ninguna acción específica en este caso.
     */
    @Override
    public void initialize() {
        // No se requiere lógica adicional al comenzar.
    }

    /**
     * Se ejecuta repetidamente mientras el comando está activo.
     * Ajusta la velocidad del intake del subsistema Coral a intakeVelocity.
     */
    @Override
    public void execute() {
        coralSubSystem.enableCoralIntake(intakeVelocity);
    }

    /**
     * Se llama cuando el comando termina o es interrumpido, deteniendo el intake.
     *
     * @param interrupted Indica si el comando finalizó de forma normal o fue interrumpido.
     */
    @Override
    public void end(boolean interrupted) {
        coralSubSystem.stopCoralIntake();
    }

    /**
     * Retorna siempre false, ya que este comando no finaliza por sí solo.
     *
     * @return false siempre.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
