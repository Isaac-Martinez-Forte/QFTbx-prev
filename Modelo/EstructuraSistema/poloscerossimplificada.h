#ifndef POLOSCEROSSIMPLIFICADO_H
#define POLOSCEROSSIMPLIFICADO_H

#include "funciontransferencia.h"
#include "mpParser.h"

class PolosCerosSimplificada : public FuncionTransferenciaSimplificada
{
public:
  
  
  /**
    * @fn PolosCeros
    * @brief Constructor de la clase que tiene como parámetros los datos básicos de una planta de este tipo.
    * 
    * @param nombre de la planta.
    * @param numerador de la planta
    * @param denominador de la planta
    * @param k ganancia asociada a la planta
    * @param retardo asociado a la planta
   */
  
    PolosCerosSimplificada(QVector <qreal> * numeradorSup, QVector <qreal> * numeradorInf, QVector <qreal> * denominadorSup, QVector <qreal> * denominadorInf, 
            qreal kMax, qreal kMin);
    
    
  /**
    * @fn PolosCeros
    * @brief Constructor de la clase que tiene como parámetro un objeto ya creado del mismo tipo.
    * 
    * @param datos objeto del mismo tipo creado anteriormente.
    */
    
    PolosCerosSimplificada(PolosCerosSimplificada * datos);

    virtual SistemaSimplificado * invoke (QVector <qreal> * numeradorSup, QVector <qreal> * numeradorInf, QVector <qreal> * denominadorSup, QVector <qreal> * denominadorInf, 
            qreal kMax, qreal kMin) = 0;
    
    
    /**
     * @fn ~PolosCeros
     * @brief Destructor de la clase.
     */
    
    ~PolosCerosSimplificada();
    
    virtual std::complex <qreal> getPunto (QVector <qreal> * numerador, QVector <qreal> * denominador,
                                           qreal k, qreal ret, qreal omega) = 0;


    virtual std::complex <qreal> getPuntoNume(QVector <qreal> * nume, qreal omega) = 0;

    virtual std::complex <qreal> getPuntoDeno(QVector <qreal> * deno, qreal omega) = 0;
    
   /**
    * @fn getClass
    * @brief Función virtual pura que retorna la clase de la instancia.
    * 
    * @return cadena con la clase de la instancia.
    */

    virtual tipo_planta getClass() = 0;

    virtual Sistema * toSistema() = 0;

};

#endif // POLOSCEROS_H
 
