#ifndef FUNCIONTRANSFERENCIASIMPLIFICADA_H
#define FUNCIONTRANSFERENCIASIMPLIFICADA_H

#include "sistema.h"
#include <QVector>
#include "mpParser.h"

class FuncionTransferenciaSimplificada : public SistemaSimplificado
{
public:
  
    FuncionTransferenciaSimplificada(QVector <qreal> * numeradorSup, QVector <qreal> * numeradorInf, QVector <qreal> * denominadorSup, QVector <qreal> * denominadorInf, 
            qreal kMax, qreal kMin);
    
 
    FuncionTransferenciaSimplificada (FuncionTransferenciaSimplificada *datos);


    virtual SistemaSimplificado * invoke (QVector <qreal> * numeradorSup, QVector <qreal> * numeradorInf, QVector <qreal> * denominadorSup, QVector <qreal> * denominadorInf, 
            qreal kMax, qreal kMin) = 0;
    
    
    /**
     * @fn ~FuncionTransferencia
     * @brief Destructor de la clase.
     */

    ~FuncionTransferenciaSimplificada();

    virtual std::complex <qreal> getPunto (QVector <qreal> * numerador, QVector <qreal> * denominador,
                                           qreal k, qreal ret, qreal omega) = 0;

    virtual std::complex <qreal> getPuntoNume(QVector <qreal> * nume, qreal omega) = 0;

    virtual std::complex <qreal> getPuntoDeno(QVector <qreal> * deno, qreal omega) = 0;

    
    QVector <qreal> * getDenominadorSup();
    QVector <qreal> * getDenominadorInf();
     
    QVector <qreal> * getNumeradorSup();
    QVector <qreal> * getNumeradorInf();

    qreal getKMax ();
    qreal getKMin ();
    
    void borrar();

    virtual tipo_planta getClass () = 0;

    SistemaSimplificado * clone ();

    virtual Sistema * toSistema() = 0;

protected:
    qreal kMax;
    qreal kMin;
    
    QVector <qreal> * numeradorSup; 
    QVector <qreal> * numeradorInf; 
    QVector <qreal> * denominadorSup; 
    QVector <qreal> * denominadorInf;

    bool b = true;

};

#endif // FUNCIONTRANSFERENCIA_H
 
