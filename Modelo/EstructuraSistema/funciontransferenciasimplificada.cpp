#include "funciontransferenciasimplificada.h"

using namespace std;
using namespace mup;

FuncionTransferenciaSimplificada::FuncionTransferenciaSimplificada(QVector <qreal> * numeradorSup, QVector <qreal> * numeradorInf, QVector <qreal> * denominadorSup, QVector <qreal> * denominadorInf,
        qreal kMax, qreal kMin) :
SistemaSimplificado() {
    this->numeradorInf = numeradorInf;
    this->numeradorSup = numeradorSup;

    this->denominadorInf = denominadorInf;
    this->denominadorSup = denominadorSup;

    this->kMax = kMax;
    this->kMin = kMin;
}

FuncionTransferenciaSimplificada::FuncionTransferenciaSimplificada(FuncionTransferenciaSimplificada * datos) :
FuncionTransferenciaSimplificada(datos->numeradorSup, datos->numeradorInf, datos->denominadorSup, datos->denominadorInf, datos->kMax, datos->kMin) {
}

FuncionTransferenciaSimplificada::~FuncionTransferenciaSimplificada() {

    if (b) {
        numeradorInf->clear();
        numeradorSup->clear();
        denominadorInf->clear();
        denominadorSup->clear();
    }
}

QVector <qreal> * FuncionTransferenciaSimplificada::getDenominadorSup() {
    return denominadorSup;
}

QVector <qreal> * FuncionTransferenciaSimplificada::getDenominadorInf() {
    return denominadorInf;
}

QVector <qreal> * FuncionTransferenciaSimplificada::getNumeradorSup() {
    return numeradorSup;
}

QVector <qreal> * FuncionTransferenciaSimplificada::getNumeradorInf() {
    return numeradorInf;
}

qreal FuncionTransferenciaSimplificada::getKMax() {
    return kMax;
}

qreal FuncionTransferenciaSimplificada::getKMin() {
    return kMin;
}

void FuncionTransferenciaSimplificada::borrar(){
    b = false;
}

SistemaSimplificado * FuncionTransferenciaSimplificada::clone() {

    QVector <qreal> * ns = new QVector <qreal> ();
    QVector <qreal> * ds = new QVector <qreal> ();
    QVector <qreal> * ni = new QVector <qreal> ();
    QVector <qreal> * di = new QVector <qreal> ();

    for (int i = 0; i < numeradorInf->size(); i++) {
        ns->append(numeradorSup->at(i));
        ni->append(numeradorInf->at(i));
    }

    for (int i = 0; i < denominadorInf->size(); i++) {
        ds->append(denominadorSup->at(i));
        di->append(denominadorInf->at(i));
    }

    return this->invoke(ns, ni, ds, di, kMax, kMin);
}
 
