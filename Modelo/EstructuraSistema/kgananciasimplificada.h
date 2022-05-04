#ifndef KGANANCIASIMPLIFICADA_H
#define KGANANCIASIMPLIFICADA_H

#include "mpParser.h"

#include <QString>

class KGananciaSimplificada : public PolosCerosSimplificada
{

public:
  
    KGananciaSimplificada(QVector <qreal> * numeradorSup, QVector <qreal> * numeradorInf, QVector <qreal> * denominadorSup, QVector <qreal> * denominadorInf, 
            qreal kMax, qreal kMin);
    
    SistemaSimplificado * invoke (QVector <qreal> * numeradorSup, QVector <qreal> * numeradorInf, QVector <qreal> * denominadorSup, QVector <qreal> * denominadorInf, 
            qreal kMax, qreal kMin);
    
    ~KGananciaSimplificada();
    
    std::complex <qreal> getPunto (QVector <qreal> * numerador, QVector <qreal> * denominador,
                                           qreal k, qreal ret, qreal omega);


    std::complex <qreal> getPuntoNume(QVector <qreal> * nume, qreal omega);

    std::complex <qreal> getPuntoDeno(QVector <qreal> * deno, qreal omega);
    
    tipo_planta getClass();

    Sistema * toSistema();
};


#endif // KGANANCIA_H
 
