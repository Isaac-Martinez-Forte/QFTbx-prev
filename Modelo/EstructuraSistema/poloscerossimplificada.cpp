#include "poloscerossimplificada.h"

using namespace std;

PolosCerosSimplificada::PolosCerosSimplificada(QVector <qreal> * numeradorSup, QVector <qreal> * numeradorInf, QVector <qreal> * denominadorSup, QVector <qreal> * denominadorInf,
        qreal kMax, qreal kMin) :
FuncionTransferenciaSimplificada(numeradorSup, numeradorInf, denominadorSup, denominadorInf, kMax, kMin) {
}

PolosCerosSimplificada::PolosCerosSimplificada(PolosCerosSimplificada *datos) : FuncionTransferenciaSimplificada(datos) {
}

PolosCerosSimplificada::~PolosCerosSimplificada() {
} 
