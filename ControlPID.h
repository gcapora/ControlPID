/****************************************************************************************
  ControlPID.h
-----------------------------------------------------------------------------------------
  Descripción:
           Objeto de control PID. 
           Limita efecto enrole mediante saturación y bloqueo de integración.
-----------------------------------------------------------------------------------------
  Autor:   Guillermo Caporaletti <gfcaporaletti@undav.edu.ar>
           Sistemas de Control Automático (SCA)
           Universidad Nacional de Avellaneda (UNDAV)
  Fecha:   Diciembre 2021
  Version: 1.0
****************************************************************************************/

#ifndef CONTROLPID_h
#define CONTROLPID_h
#include "Arduino.h"

/***************************************************************************************/

class controlPID                     // Objeto para control Proporcional-Integral-Derivativo (PID)
{  private:
      float Salida;                  // La señal de control que va al actuador
                                     // o potencia de salida (sin asignar unidades)
      float Proporcional;            // Componente proporcional de la salida (sin asignar unidades)
      float Integral;                // Componente integral 
      float Derivativo;              // Componente derivativa
      float Kp;                      // Constante proporcional (sin asignar unidades)
      float Ti;                      // Tiempo de integración (en segundos)
      float Td;                      // Tiempo para la componente derivativa (en segundos)
      unsigned long TiempoAnterior;  // Tiempo de la medición anterior utilizando micros (en microsegundos)
      float ErrorAnterior;           // Señal de error anterior 
      boolean LimitaSalida;          // Indica si establecimos límites superior e inferior a la salida
      boolean LimitaIntegral;        // Indica si establecimos límites superior e inferior en la integral del PID
                                     // (en caso true, son los mismos límites que la salida) 
      boolean CondicionaIntegral;    // Condiciona la ejecución de la integral a que la salida no esté saturada.
      float SalidaMax;               // Límite superior de la salida (y de la integral)
      float SalidaMin;               // Límite inferior de la salida
      const float MILLON=1e6;        // Constante para convertir micros() a segundos.
      
   public:
      controlPID(float KP, float TI, float TD);            // Constructor con lo mínimo:
                                                           // KP: Constante de proporcionalidad (puede ser negativo)
                                                           // TI: Tiempo de integración (si es 0, no integra)
                                                           // TD: Tiempo de derivación (si es 0 no deriva)
      void ConfigurarPID(float KP, float TI, float TD);    // Mismos parámetros que el constructor.
                                                           // Sirve para cambiar configuración inicial.
      boolean LimitarSalida(boolean RESPUESTA, float SMIN, float SMAX);  // Configura los límites de salida.
                                                           // e indica si están activados.
      boolean LimitarSalida(boolean RESPUESTA);            // Activa o desactiva los límites de salida
                                                           // e indica si el límite de salida está activado.
                                                           // No permite activar límites si antes no fueron establecidos.
      boolean LimitarSalida();                             // Indica si el límite de salida está activado.
      boolean LimitarIntegral(boolean RESPUESTA);          // Activa o desactiva el límite de integración
                                                           // e indica si el límite de integración está activado.
      boolean LimitarIntegral();                           // Me indica si el límite de integración está activado.
      boolean CondicionarIntegral(boolean RESPUESTA);      // Activa o desactiva el condicional de integración 
                                                           // e indica si está activado.
                                                           // El condicional implicaque no integrará mientras la salida esté saturada.
      boolean CondicionarIntegral();                       // Me indica si el condicional de integración está activado.
      float Controlar(float ERROR);                        // Calcula señal de control (salida) en función del error.
      void Apagar();                                       // Apaga el PID y resetea valores.
                                                           // No se modifican los valores de KP, TI y TD.
                                                           // Tampoco los límites pre establecidos.
      float ObtenerIntegral(); 
      float ObtenerProporcional(); 
      float ObtenerDerivativo(); 
      float ObtenerSalida();
};

/***************************************************************************************/

