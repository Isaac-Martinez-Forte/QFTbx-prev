#ifndef FUNCIONESCOMUNES_H
#define FUNCIONESCOMUNES_H


#include <QVector>
#include <QPointF>

#include "Modelo/EstructuraSistema/sistema.h"
#include "Modelo/Herramientas/tools.h"
#include "GUI/viewboundreun.h"
#include "Modelo/EstructurasDatos/datosbound.h"
#include "Modelo/LoopShaping/NaturalIntervalExtension/natural_interval_extension.h"
#include "Modelo/LoopShaping/DeteccionViolacionBoundaries/deteccionviolacionboundaries.h"
#include "Modelo/LoopShaping/EstructuraDatos/listaordenada.h"

#include "cinterval.hpp"
#include <complex>

using namespace tools;
using namespace cxsc;;

namespace FC {

struct return_bisection {
    Sistema * v1;
    Sistema * v2;
    bool descartado;
};

enum diagrama {Nichol = false, Nyquist = true};

inline Sistema * guardarControlador(Sistema *controlador, bool x) {


    QVector <Var *> * nume = controlador->getNumerador();
    QVector <Var *> * numerador = new QVector <Var *> ();

    foreach (Var * v, *nume) {
        if (v->isVariable()){
            if (x){
                numerador->append(new Var (v->getRango().x()));
            }else {
                numerador->append(new Var (v->getRango().y()));
            }
        } else {
            numerador->append(new Var (v->getNominal()));
        }
    }

    QVector <Var *> * deno = controlador->getDenominador();
    QVector <Var *> * denominador = new QVector <Var *> ();

    foreach (Var * v, *deno) {
        if (v->isVariable()){
            if (x){
                denominador->append(new Var (v->getRango().x()));
            }else {
                denominador->append(new Var (v->getRango().y()));
            }
        } else {
            denominador->append(new Var (v->getNominal()));
        }
    }

    Var * k;

    if (x){
        k = new Var (controlador->getK()->getRango().x());
    } else {
        k = new Var (controlador->getK()->getRango().y());
    }



    Sistema * s = controlador->invoke(controlador->getNombre(), numerador, denominador,
                                      k, new Var ((qreal) 0));

    delete controlador;

    return s;
}

inline bool if_less_epsilon(Sistema * controlador, qreal epsilon, QVector <qreal> * omega,
                            Natura_Interval_extension *conversion, QVector <complex> * plantas_nominales) {

    cinterval box;
    for (qint32 i = 0; i < omega->size(); i++){
        box = conversion->get_box(controlador, omega->at(i), plantas_nominales->at(i), false);

        if (((SupIm(box) - InfIm(box)) >= epsilon) || ((SupRe(box) - InfRe(box)) >= epsilon)) {
            return false;
        }
    }

    return true;
}

inline ViewBoundReun * mostrar_diagrama(QVector <QVector<QPointF> * > *vector, QVector <qreal> * omega, DatosBound * boundaries) {
    ViewBoundReun * view = new ViewBoundReun();

    view->setDatos(boundaries->getBoundariesReun(), omega);

    view->mostrar_diagrama();

    qint32 contador = 0;

    foreach(QVector <QPointF> * vec, *vector) {
        view->dibujar_cuadro(vec->at(0), vec->at(1), vec->at(2), vec->at(3), contador);
        contador++;
    }

    /*view->exec();

        delete view;*/

    return view;
}

inline void mostrar_diagrama2 (QVector <QVector<QPointF> * > *vector, QVector <qreal> * omega, DatosBound * boundaries, ViewBoundReun * view) {
    //ViewBoundReun * view = new ViewBoundReun();

    /*view->setDatos(boundaries->getBoundariesReun(), omega);

        view->mostrar_diagrama();*/

    qint32 contador = 0;

    foreach(QVector <QPointF> * vec, *vector) {
        view->dibujar_cuadro2(vec->at(0), vec->at(1), vec->at(2), vec->at(3), contador);
        contador++;
    }

    view->exec();

    delete view;
}

inline void mostrar_diagramaBox(QVector<QPointF> * caja, QVector <qreal> * omega, DatosBound * boundaries) {
    ViewBoundReun * view = new ViewBoundReun();

    view->setDatos(boundaries->getBoundariesReun(), omega);

    view->mostrar_diagrama();

    view->dibujar_cuadro(caja->at(0), caja->at(1), caja->at(2), caja->at(3), 0);

    view->exec();

    delete view;
    caja->clear();
}

//Función que divide la caja en dos.

inline return_bisection split_box_bisection(Sistema *current_controlador) {

    QVector <Var *> * numerador = current_controlador->getNumerador();
    QVector <Var *> * denominador = current_controlador->getDenominador();

    QVector <Var *> * numeradorCopia = new QVector <Var *> ();
    QVector <Var *> * denominadorCopia = new QVector <Var *> ();

    Var * k = current_controlador->getK();
    Var * ret = current_controlador->getRet();

    QString nombre = current_controlador->getNombre();

    //Variables contador;
    qint32 mayor_pos = -1;
    qreal mayor_rango = -1;

    //Variables auxiliares
    qreal lon = 0;
    qreal cont = 0;

    //Sistemas hijos creados

    Sistema * v1, * v2;
    struct FC::return_bisection retur;


    //Bucle del numerador
    Var * v;
    for (qint32 i = 0; i < numerador->size(); i++) {
        v = numerador->at(i);
        numeradorCopia->append(v->clone());
        if (v->isVariable()) {

            lon = v->getRango().y() - v->getRango().x();

            if (lon > mayor_rango) {
                mayor_pos = cont;
                mayor_rango = lon;
            }
        }
        cont++;
    }

    //Bucle del denominador
    for (qint32 i = 0; i < denominador->size(); i++) {
        v = denominador->at(i);
        denominadorCopia->append(v->clone());
        if (v->isVariable()) {

            lon = v->getRango().y() - v->getRango().x();

            if (lon > mayor_rango) {
                mayor_pos = cont;
                mayor_rango = lon;
            }
        }
        cont++;
    }

    //Estudiamos la k
    if (k->isVariable()) {

        lon = k->getRango().y() - k->getRango().x();

        if (lon > mayor_rango) {
            mayor_pos = -1;
            mayor_rango = lon;
        }
    }


    if (mayor_pos == -1) {
        qreal dis = k->getRango().x();
        Var * k1 = new Var("kv", QPointF(dis, dis + (mayor_rango / 2)), dis, k->getGranularidad());
        dis += mayor_rango / 2;
        Var * k2 = new Var("kv", QPointF(dis, k->getRango().y()), dis, k->getGranularidad());

        delete k;

        v1 = current_controlador->invoke(nombre, numerador, denominador, k1, ret);
        v2 = current_controlador->invoke(nombre, numeradorCopia, denominadorCopia, k2, ret->clone());
    } else if (mayor_pos < numerador->size()) {

        Var * variable = numerador->at(mayor_pos);

        qreal dis = variable->getRango().x();

        numeradorCopia->replace(mayor_pos, new Var("", QPointF(dis, dis + mayor_rango / 2), dis, variable->getGranularidad()));

        dis += mayor_rango / 2;
        numerador->replace(mayor_pos, new Var("", QPointF(dis, variable->getRango().y()), dis,
                                              variable->getGranularidad()));


        v1 = current_controlador->invoke(nombre, numeradorCopia, denominadorCopia, k->clone(), ret->clone());
        v2 = current_controlador->invoke(nombre, numerador, denominador, k, ret);

        delete variable;

    } else {
        mayor_pos -= numerador->size();

        Var * variable = denominador->at(mayor_pos);
        qreal dis = variable->getRango().x();

        denominadorCopia->replace(mayor_pos, new Var("", QPointF(dis, dis + mayor_rango / 2), dis, variable->getGranularidad()));

        dis += mayor_rango / 2;
        denominador->replace(mayor_pos, new Var("", QPointF(dis, variable->getRango().y()), dis,
                                                variable->getGranularidad()));

        v1 = current_controlador->invoke(nombre, numeradorCopia, denominadorCopia, k->clone(), ret->clone());
        v2 = current_controlador->invoke(nombre, numerador, denominador, k, ret);

        delete variable;

    }


    retur.v1 = v1;
    retur.v2 = v2;
    retur.descartado = false;

    current_controlador->borrar();

    delete current_controlador;

    return retur;
}

};



#endif
