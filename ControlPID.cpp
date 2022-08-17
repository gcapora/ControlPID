/***********************************************************************************
  ControlPID.cpp
-----------------------------------------------------------------------------------
  Descripción:
           Objeto de control PID. 
           Limita efecto enrole mediante saturación y bloqueo de integración.
-----------------------------------------------------------------------------------
  Autor:   Guillermo Caporaletti <gfcaporaletti@undav.edu.ar>
           Sistemas de Control Automático (SCA)
           Universidad Nacional de Avellaneda (UNDAV)
  Fecha:   Diciembre 2021
  Version: 1.0
***********************************************************************************/

#include "Arduino.h"
#include "ControlPID.h"

/**************************************************************************************/

controlPID::controlPID(float KP, float TI, float TD)                   
// Constructor: incluye configuración inicial del PID y valores predeterminados.
{  // Kp puede ser negativo (esto último podría servir para controlar una planta cuya salida 
   //                        tienda a bajar cuando aumente la señal de control. Ej.: heladera.) 
   // Si Ti=0, el PID no lo tomará en cuenta
   // Si Td=0, el PID no lo tomará en cuenta
   // Inicializa integración e impone false en límites y condicional.
   ConfigurarPID(KP, TI, TD);
   LimitaSalida=false;
   LimitaIntegral=false;
   CondicionaIntegral=false;
}
//-------------------------------------------------------------------------------------

void controlPID::ConfigurarPID(float KP, float TI, float TD)
// Configura las constantes básicas del control PID 
// Sirve para cambiar configuración inicial, sin modificar límites y condicional.
{  // Kp puede ser negativo (esto último podría servir para controlar una planta cuya salida 
   //                        tienda a bajar cuando aumente la señal de control. Ej.: heladera.) 
   // Si Ti=0, el PID no lo tomará en cuenta
   // Si Td=0, el PID no lo tomará en cuenta
   // Resetea valores de integración.
   Kp=KP;
   Ti=TI;
   Td=TD;
   TiempoAnterior=0;
   ErrorAnterior=0;
   Integral=0;
   //LimitaSalida=false;
   //LimitaIntegral=false;   
   //CondicionaIntegral=false;
}
//-------------------------------------------------------------------------------------

boolean controlPID::LimitarSalida()
// Devuelve el valor de la variable privada LimitaSalida
// que indica si nuestro PID está configurado para limitar su salida.
{  return LimitaSalida;
}
//-------------------------------------------------------------------------------------

boolean controlPID::LimitarSalida(boolean RESPUESTA)
// Configura si limitará la salida...
// No permite activar límites si antes no fueron establecidos.
{  LimitaSalida=RESPUESTA;
   if (SalidaMax==0 && SalidaMin==0) {
     // ...no voy a limitar porque no tengo límites definidos.
     LimitaSalida=false;
   }
   return LimitaSalida;
}//-------------------------------------------------------------------------------------

boolean controlPID::LimitarSalida(boolean RESPUESTA, float SMIN, float SMAX)
// Configura si limitará la salida entre SMAX y SMIN.
// Se puede establecer los límites pero no activarlos aún.
// No activa con SMIN=SMAX.
// No activa si SMIN>SMAX.
{  SalidaMax=SMAX;
   SalidaMin=SMIN;
   LimitaSalida=RESPUESTA;
   // La forma de desactivar este límite es:
   // 1) Volviendo a llamar esta función con RESPUESTA=false
   // 2) Llamando a LimitarSalida(false)
   // También se pueden poner límites muy grandes.
   if (SalidaMax==SalidaMin) {
     // ...no voy a limitar porque no tengo límites definidos.
     LimitaSalida=false;
   }
   if (SalidaMin>SalidaMax) {
     // ...no voy a limitar porque están mal configurados.
     LimitaSalida=false;
   }
   return LimitaSalida;
}
//-------------------------------------------------------------------------------------

boolean controlPID::LimitarIntegral()
// Devuelve el valor de la variable privada LimitaIntegral
// que indica si nuestro PID está configurado para limitar la integral.
{  return LimitaIntegral;
}
//-------------------------------------------------------------------------------------

boolean controlPID::LimitarIntegral(boolean RESPUESTA)
// Configura los límites al integrador entre los mismos márgenes de la salida.
// Para establecer el límite, previamente se debe haber configurado los límites de salida.
{  LimitaIntegral=RESPUESTA;  // Puede ser true o false
   if (SalidaMax==SalidaMin) {
     // ...no voy a limitar porque no tengo límites definidos.
     LimitaIntegral=false;
   }
   return LimitaIntegral;
}
//-------------------------------------------------------------------------------------

boolean controlPID::CondicionarIntegral()
// Devuelve el valor de CondicionaIntegral.
{  return CondicionaIntegral;
}
//-------------------------------------------------------------------------------------

boolean controlPID::CondicionarIntegral(boolean RESPUESTA)
// Establece si debo pausar la integración cuando la salida está saturada.
// Deben haberse preestablecido los límites de salida.
{  
   CondicionaIntegral = RESPUESTA;
   if (!LimitaSalida) CondicionaIntegral=false;
   return CondicionaIntegral;
}
//-------------------------------------------------------------------------------------

float controlPID::Controlar(float ERROR)
// Calcula Salida en función de la señal error y los parámetros del PID
{  unsigned long TiempoActual = micros(); // Tomo tiempo actual para comparar 
                                          // con la medida anterior
   boolean SalidaEstaSaturada = false;
   
   // PROPORCIONAL --------------------------------------------------------------
   Proporcional = Kp*ERROR;

   // DERIVATIVO ----------------------------------------------------------------
   if (TiempoAnterior>0 && Td!=0) {  
      // Dos condiciones para componente derivativa:
      // 1) Que no sea el primer cálculo y 2) Td seteado
      Derivativo = Kp*Td*(ERROR-ErrorAnterior)*MILLON / (TiempoActual-TiempoAnterior);
   } else { 
      Derivativo = 0;
   }

   // ¿Debo saturar salida? -----------------------------------------------------
   Salida = Proporcional + Integral + Derivativo;
   if (LimitaSalida && ((Salida > SalidaMax) || (Salida<SalidaMin))) {
      // Debo saturar la salida...
      SalidaEstaSaturada = true;
   }
   
   // INTEGRAL ------------------------------------------------------------------
   if (TiempoAnterior>0 && Ti!=0) {
      // Cumplidas las dos primeras condiciones para integral el error:
      
      if (!CondicionaIntegral || !SalidaEstaSaturada) {
        // Si no está configurada la condición o si no está salutarda la salida, 
        // puedo hacer la integral:
        Integral += Kp * (ERROR+ErrorAnterior) * (TiempoActual-TiempoAnterior) / (2*Ti*MILLON);
      }

      if (LimitaIntegral) {
        // Debo saturar la integral:
        Integral = min(Integral, SalidaMax);
        Integral = max(Integral, SalidaMin);
      }
   }

   // Termina componente integral ----------------------------------------------
   
   // Cáculo final completo: 
   Salida = Proporcional + Integral + Derivativo;

   if (LimitaSalida) {
      // Debo saturar la salida:
      Salida = min(Salida, SalidaMax);
      Salida = max(Salida, SalidaMin);
   }
   TiempoAnterior = TiempoActual;
   ErrorAnterior = ERROR;
   return Salida;
   // Termina funcion PID ------------------------------------------------------
}
//-------------------------------------------------------------------------------------

float controlPID::ObtenerIntegral()
{
   return Integral; 
}
//-------------------------------------------------------------------------------------

float controlPID::ObtenerProporcional()
{
   return Proporcional;
}
//-------------------------------------------------------------------------------------

float controlPID::ObtenerDerivativo()
{
   return Derivativo;
}
//-------------------------------------------------------------------------------------

float controlPID::ObtenerSalida()
{
   return Salida;
}
//-------------------------------------------------------------------------------------

void controlPID::Apagar()
{
   TiempoAnterior=0;
   ErrorAnterior=0;
   Integral=0;   
   Proporcional=0;
   Derivativo=0; 
}
