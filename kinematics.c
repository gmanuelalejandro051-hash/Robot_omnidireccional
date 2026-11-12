#include <stdio.h>
#include <math.h>
#include "kinematics.h"

#define PI 3.14159265358979323846

#define WHEEL_RADIUS   0.061   // r  (meters)
#define ROBOT_RADIUS   0.09   // R  (meters, center to wheel)
#define DT             0.01   // control loop period (seconds)

// ── Wheel indices ────────────────────────────────────────────────────────────
#define W1  0   // α = 90°
#define W2  1   // α = 210°
#define W3  2   // α = 330°


// ── Pre-computed Jacobian rows ───────────────────────────────────────────────
// J[i] = { -sin(alpha_i), cos(alpha_i), R }
static const double alpha[3] = {
    90.0  * PI / 180.0,   // W1
    210.0 * PI / 180.0,   // W2
    330.0 * PI / 180.0    // W3
};



//esta funcion recibe como parametros los datos enviados por mqtt de la gui hecha en python. 
//me regresa los setpoint de los perfiles de velocidad para cada motor en la estructura perfil_velocidad_t
perfil_velocidad_t kinematica_inversa (velocidad_t velocidad)
{
        perfil_velocidad_t motor; 
            for(int i =0 ; i<3 ; i++)
            {
                double sa = sin(alpha[i]); 
                double ca = cos(alpha[i]); 

                motor.perfil_velocidad[i] = (1.0 / WHEEL_RADIUS) *
                  (-sa * velocidad.velocidad_x + ca * velocidad.velocidad_y + ROBOT_RADIUS * velocidad.velocidad_angular);

                //Ecuacion planteada 
                //wi​=r1​(−sin(αi​)vx​+cos(αi​)vy​+Lω).
                //nos permite obtener el perfil de velocidad para cada uno de los motores dada velocidad en x,y,velocidad angular.
                //pagina 31 motion modeling for mobile robots. 

            }
            return motor; //regresa velocidad angular para cada una de las ruedas. 

}


//Basicamente nos permite comparar el desplazamiento en x y y calculado de la cinematica inversa con el real. 
//para implementar control feedforward, utilizamos kinematica directa como retroalimentacion para obtener la velocidad actual del robot a partir de los perfiles de velocidad de las ruedas.
//Dada la velocidad angular de cada rueda, puedo calcular la velocidad lineal en x, y y la velocidad angular del robot.
//los parametros que regresa esta funcion me sirven para comparar con los setpoints de velocidad que se le dieron a la funcion de kinematica inversa y asi implementar el control feedforward.

velocidad_t kinematica_directa (perfil_velocidad_t motor)
{
    double vx = 0.0, vy = 0.0, omega = 0.0;

    for (int i = 0; i < 3; i++) {
        vx    += -sin(alpha[i]) * motor.perfil_velocidad[i];
        vy    +=  cos(alpha[i]) * motor.perfil_velocidad[i];
        omega +=  motor.perfil_velocidad[i];           // J[i][2] = R, factored out below
    }

    const double gain = WHEEL_RADIUS * (2.0 / 3.0);

    velocidad_t bv = {
        .velocidad_x    = gain * vx,
        .velocidad_y    = gain * vy,
        .velocidad_angular = gain * omega / ROBOT_RADIUS   // divide out the R factor
    };

    return bv;
}

// ── Odometry (dead-reckoning pose update) ────────────────────────────────────
void update_odometry(Pose *pose, velocidad_t bv, double dt)
{
    double ct = cos(pose->theta);
    double st = sin(pose->theta);
    pose->x     += (bv.velocidad_x * ct - bv.velocidad_y * st) * dt;
    pose->y     += (bv.velocidad_x * st + bv.velocidad_y * ct) * dt;
    pose->theta += bv.velocidad_angular * dt;
}
