#include "kgananciasimplificada.h"

using namespace std;
using namespace mup;


KGananciaSimplificada::KGananciaSimplificada(QVector <qreal> * numeradorSup, QVector <qreal> * numeradorInf, QVector <qreal> * denominadorSup, QVector <qreal> * denominadorInf, 
            qreal kMax, qreal kMin):
    PolosCerosSimplificada(numeradorSup, numeradorInf, denominadorSup, denominadorInf, kMax, kMin)
{
}

KGananciaSimplificada::~KGananciaSimplificada(){
}

SistemaSimplificado * KGananciaSimplificada::invoke (QVector <qreal> * numeradorSup, QVector <qreal> * numeradorInf, QVector <qreal> * denominadorSup, QVector <qreal> * denominadorInf, 
            qreal kMax, qreal kMin){
    return new KGananciaSimplificada(numeradorSup, numeradorInf, denominadorSup, denominadorInf, kMax, kMin);
}



std::complex <qreal> KGananciaSimplificada::getPunto (QVector <qreal> * numerador, QVector <qreal> * denominador,
                            qreal k, qreal ret, qreal omega){
    qint32 sizeDen = denominador->size();
    qint32 sizeNum = numerador->size();

    QString es;


    es += QString::number(k) + "*(";


    if (numerador->isEmpty()){
        es += "1) / (";
    } else {
        for (qint32 i = 0; i < sizeNum-1; i++){

            es += "(("+ QString::number(omega) + "*i) +" + QString::number(numerador->at(i)) + ") *";
        }

        es += "((" + QString::number(omega) + "*i) + " + QString::number(numerador->last()) + ")) / (";
    }


    if (denominador->isEmpty()){
        es += "1)";
    } else {
        for (qint32 i = 0; i < sizeDen-1; i++){

            es += "(("+ QString::number(omega) + "*i) + " + QString::number(denominador->at(i)) + ") *";
        }

        es += "(("+ QString::number(omega) + "*i) + " + QString::number(denominador->last()) + "))";
    }

    if (ret != 0){
        es += "* e^(i*" + QString::number(omega) + "*" + QString::number(ret) +")";
    }

    mup::ParserX p (mup::pckALL_COMPLEX);

    p.SetExpr(es.toStdString());

    return p.Eval().GetComplex();

}



SistemaSimplificado::tipo_planta KGananciaSimplificada::getClass(){
    return k_ganancia;
}


std::complex <qreal> KGananciaSimplificada::getPuntoNume(QVector <qreal> * nume, qreal omega){

    if (nume->isEmpty()){
        return std::complex <qreal>(1);
    }

    qint32 sizeNum = nume->size();
    QString es = "(";

    for (qint32 i = 0; i < sizeNum-1; i++){

        es += "(("+ QString::number(omega) + "*i) +" + QString::number(nume->at(i)) + ") *";
    }

    es += "((" + QString::number(omega) + "*i) + " + QString::number(nume->last()) + "))";


    mup::ParserX p (mup::pckALL_COMPLEX);

    p.SetExpr(es.toStdString());

    return p.Eval().GetComplex();
}

std::complex <qreal> KGananciaSimplificada::getPuntoDeno(QVector <qreal> * deno, qreal omega){

    if (deno->isEmpty()){
        return std::complex <qreal>(1);
    }

    qint32 sizeDen = deno->size();
    QString es = "(";

    for (qint32 i = 0; i < sizeDen-1; i++){

        es += "(("+ QString::number(omega) + "*i) + " + QString::number(deno->at(i)) + ") *";
    }

    es += "(("+ QString::number(omega) + "*i) + " + QString::number(deno->last()) + "))";

    mup::ParserX p (mup::pckALL_COMPLEX);

    p.SetExpr(es.toStdString());

    return p.Eval().GetComplex();
}

 
