package frc.robot.Intakes.Algae;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeConstants;

/**
 * AlgaePivotCmd es un comando continuo que controla la inclinación o pivote
 * del mecanismo de intake (Algae). Permite mover el pivote a una velocidad
 * específica y en una dirección determinada mediante la bandera isInverted.
 * 
 * Funcionalidades:
 * - En initialize(), se calcula la velocidad final según la constante pivotMotorVelocity
 *   (definida en AlgaeConstants) y el estado de isInverted.
 * - En execute(), se llama al subsistema para aplicar la velocidad calculada al pivote.
 * - En end(), se detiene el motor del pivote.
 * - El método isFinished() retorna siempre false, por lo que este comando no termina
 *   automáticamente, sino que normalmente se interrumpe desde otro comando o evento.
 * 
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.1
 */
public class AlgaePivotCmd extends Command {

    /**
     * Indica si la rotación del pivote se invierte.
     */
    private boolean isInverted;

    /**
     * Subsistema que controla el mecanismo Algea (intake) y sus partes móviles, incluido el pivote.
     */
    private AlgaeSubSystem algeaSubSystem;

    /**
     * Velocidad final calculada para el pivote, basada en la constante pivotMotorVelocity y la bandera isInverted.
     */
    private double finalVelocity;

    /**
     * Proveedor que devuelve el valor del joystick en el eje Y.
     */
    private Supplier<Double> yJoystickSupplier, leftTriggerSupplier, RightTriggerSupplier;

    /**
     * Crea un nuevo comando para ajustar la inclinación/pivote de Algea.
     *
     * @param invertDirection  true para invertir la dirección del pivote.
     * @param algeaSubSystem   Instancia del subsistema Algea a controlar.
     * @param yJoystickSupplier Proveedor del valor del joystick en el eje Y.
     */
    public AlgaePivotCmd(
        boolean invertDirection, 
        AlgaeSubSystem algeaSubSystem, 
        Supplier<Double> yJoystickSupplier, 
        Supplier<Double> leftTriggerSupplier, 
        Supplier<Double> rightTriggerSupplier
        ) {
        this.algeaSubSystem = algeaSubSystem;
        this.isInverted = invertDirection;
        this.yJoystickSupplier = yJoystickSupplier;
        this.leftTriggerSupplier = leftTriggerSupplier;
        this.RightTriggerSupplier = rightTriggerSupplier;
        addRequirements(algeaSubSystem);
    
    }

    /**
     * Inicializa el comando.
     * (No se realiza acción adicional en este método.)
     */
    @Override
    public void initialize() {
    }

    /**
     * Ejecuta el comando de pivote:
     * - Calcula la velocidad final: (joystick * AlgaeConstants.AlgaePivotMaxVelocity), invertida si isInverted.
     * - Si el joystick está fuera del rango muerto (>=0.2 o <=-0.2), activa el pivote.
     * - De lo contrario, ajusta los pivotes izquierdo y derecho según los triggers.
     */
    @Override
    public void execute() {
        finalVelocity = (yJoystickSupplier.get() * AlgaeConstants.AlgaePivotMaxVelocity) * (isInverted ? -1 : 1);

        if (yJoystickSupplier.get() >= 0.2 || yJoystickSupplier.get() <= -0.2) {
            algeaSubSystem.enablePivot(finalVelocity);
        } else {
            algeaSubSystem.EnableLeftAlgaePivot(
                leftTriggerSupplier.get() < 0.3 ? 0 : leftTriggerSupplier.get() * AlgaeConstants.AlgaePivotMaxVelocity * -1 
            );
            algeaSubSystem.EnableRightAlgaePivot(
                RightTriggerSupplier.get() < 0.3 ? 0 : RightTriggerSupplier.get() * AlgaeConstants.AlgaePivotMaxVelocity
            );
        }
    }


    /**
     * Se llama al terminar o interrumpir el comando, deteniendo el motor del pivote.
     *
     * @param interrupted Indica si el comando terminó de forma normal o fue interrumpido.
     */
    @Override
    public void end(boolean interrupted) {
        algeaSubSystem.stopPivot();
    }

    /**
     * Retorna false para indicar que este comando no termina por sí solo.
     *
     * @return false siempre.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
