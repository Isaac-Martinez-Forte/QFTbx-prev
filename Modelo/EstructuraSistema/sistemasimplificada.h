#ifndef SISTEMASIMPLIFICADA_H
#define SISTEMASIMPLIFICADA_H

#include <QString>
#include <complex>
#include "mpParser.h"
#include "sistema.h"

class SistemaSimplificado
{
public:
  
    SistemaSimplificado ();
    
    virtual SistemaSimplificado * invoke (QVector <qreal> * numeradorSup, QVector <qreal> * numeradorInf, QVector <qreal> * denominadorSup, QVector <qreal> * denominadorInf, 
            qreal kMax, qreal kMin) = 0;
    
    
    /**
     * @fn ~Sistema
     * @brief Destructor virtual de la clase.
     */

    virtual ~SistemaSimplificado() {}

    virtual std::complex <qreal> getPunto (QVector <qreal> * numerador, QVector <qreal> * denominador,
                                           qreal k, qreal ret, qreal omega) = 0;

    virtual std::complex <qreal> getPuntoNume(QVector <qreal> * nume, qreal omega) = 0;

    virtual std::complex <qreal> getPuntoDeno(QVector <qreal> * deno, qreal omega) = 0;

    
    virtual QVector <qreal> * getDenominadorSup() = 0;
    virtual QVector <qreal> * getDenominadorInf() = 0;
     
    virtual QVector <qreal> * getNumeradorSup() = 0;
    virtual QVector <qreal> * getNumeradorInf() = 0;

    virtual qreal getKMax () = 0;
    virtual qreal getKMin () = 0;

    enum tipo_planta {formato_libre, k_ganancia, k_no_ganancia, cof_polinomios};

    virtual tipo_planta getClass () = 0;

    virtual void borrar () = 0;

    virtual SistemaSimplificado * clone () = 0;

    //virtual Sistema * toSistema() = 0;

};

#endif // SISTEMA_H
 
