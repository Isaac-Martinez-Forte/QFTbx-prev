#ifndef TRIPLETA
#define TRIPLETA

#include "cinterval.hpp"
#include "Modelo/Herramientas/tools.h"
#include "Modelo/EstructuraSistema/sistema.h"
#include "Modelo/Herramientas/tools.h"


using namespace tools;

class Tripleta {
public:

    qreal index;
    Sistema * sistema;
    flags_box flags;
    qint32 numeroFeasibles;

    QVector <qreal> * puntosCorte = NULL;
    QVector <bool> * feasible = NULL;

    QVector <struct datos_caja> * datos = NULL;
    //QVector <flags_box> * flag = NULL;


    Tripleta() {}

    Tripleta(qreal index, Sistema * sistema, flags_box flags = ambiguous) {
        this->index = index;
        this->flags = flags;
        this->sistema = sistema;
    }
    
    Tripleta(qreal index, Sistema * sistema,  QVector <struct datos_caja> * datos, qint32 nFeasible) {
        this->index = index;
        this->flags = flags;
        this->sistema = sistema;
        this->datos = datos;
        numeroFeasibles =nFeasible;
    }

    Tripleta(qreal index, Sistema * sistema, QVector <flags_box> * omegas) {
        this->index = index;
        this->sistema = sistema;
        //this->flag = omegas;
    }

    Tripleta(const Tripleta &c) { *this = c; }

    ~Tripleta() {
        if (datos != NULL){
            datos->clear();
        }

        /*if (flag != NULL){
            flag->clear();
        }*/

        if (puntosCorte != NULL){
            puntosCorte->clear();
        }

        if (feasible != NULL){
            feasible->clear();
        }
    }

    Tripleta &operator=(const Tripleta &c) {

        if(this != &c) {
            index = c.index;
            flags = c.flags;
            sistema = c.sistema;
        }

        return *this;
    }

    bool operator==(const Tripleta &c) const {
        return index == c.index;
    }
    bool operator!=(const Tripleta &c) const {
        return index != c.index;
    }
    bool operator<(const Tripleta &c) const {
        return index < c.index;
    }
    bool operator>(const Tripleta &c) const {
        return index > c.index;
    }
    bool operator<=(const Tripleta &c) const {
        return index <= c.index;
    }
    bool operator>=(const Tripleta &c) const {
        return index >= c.index;
    }

    qreal getIndex() const
    {
        return index;
    }

    void setIndex(const qreal &value)
    {
        index = value;
    }

    flags_box getFlags() const
    {
        return flags;
    }

    void setFlags(const flags_box &value)
    {
        flags = value;
    }

    qint32 getNumeroFeasibles() const
    {
        return numeroFeasibles;
    }

    void setNumeroFeasibles(const qint32 &value)
    {
        numeroFeasibles = value;
    }

    QVector<datos_caja> *getDatos() const
    {
        return datos;
    }



    void setDatos(QVector<datos_caja> *value)
    {
        datos = value;
    }

    Sistema *getSistema() const
    {
        return sistema;
    }

    void setSistema(Sistema *value)
    {
        sistema = value;
    }

    /*QVector<flags_box> *getFlag() const
    {
        return flag;
    }

    void setFlag(QVector<flags_box> *value)
    {
        flag = value;
    }
    QVector<bool> *getFeasible() const
    {
        return feasible;
    }*/

    void setFeasible(QVector<bool> *value)
    {
        feasible = value;
    }

    QVector<qreal> *getPuntosCorte() const
    {
        return puntosCorte;
    }

    void setPuntosCorte(QVector<qreal> *value)
    {
        puntosCorte = value;
    }

};

#endif // TRIPLETA
