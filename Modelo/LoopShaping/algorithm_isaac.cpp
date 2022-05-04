#include "algorithm_isaac.h"

#include<iostream>
#include<stdlib.h>
#include<time.h>


//#define sachin
#define REC_UNION
#define REC_INTER
//#define REC_FASE
#define REC_MAG
#define DETEC_CASOS
#define GANANCIA
//#define VER_ANTES
//#define COMPARACION_CAJAS
//#define Bi_INTE
//#define BE
//#define Imprimir
//#define Exportar

using namespace tools;
using namespace cxsc;

Algorithm_isaac::Algorithm_isaac() {

}

Algorithm_isaac::~Algorithm_isaac() {

}

void Algorithm_isaac::set_datos(Sistema *planta, Sistema *controlador, QVector<qreal> * omega, DatosBound *boundaries,
                                qreal epsilon, QVector<QVector<QVector<QPointF> *> *> *reunBounHash,
                                bool depuracion __attribute__((unused)), bool hilos, QVector<qreal>*radiosBoundariesMayor,
                                QVector<qreal> *radiosBoundariesMenor, QVector<QPointF> *centros, bool biseccion_avanzada, bool deteccion_avanzada, bool a) {

    this->planta = planta;
    this->controlador = controlador->clone();
    this->omega = omega;
    this->boundaries = boundaries;
    this->epsilon = epsilon;

    this->tamFas = boundaries->getTamFas() - 1;
    //this->depuracion = true;
    this->hilos = hilos;

    this->radiosBoundariesMayor = radiosBoundariesMayor;
    this->radiosBoundariesMenor = radiosBoundariesMenor;
    this->centros = centros;

    if (biseccion_avanzada) {
        split_box = &Algorithm_isaac::split_box_bisection_avanced;
    } else {
        split_box = &Algorithm_isaac::split_box_bisection;
    }

    if (deteccion_avanzada) {
        deteccionViolacion = &DeteccionViolacionBoundaries::deteccionViolacionCajaNyNi;
        Nyquist = true;

        qreal maglineal = 0;

        QVector< QVector<QPointF> * > * boun = boundaries->getBoundariesReun();

        QVector< QVector<QPointF> * > * nuevosBoundariesReun =
                new QVector< QVector<QPointF> * > ();

        QVector< QVector< QVector<QPointF> * > * > * nuevoHash_inter = new QVector< QVector< QVector<QPointF> * > * > ();

        QVector <QPointF> * datosX = new QVector <QPointF> ();
        QVector <QPointF> * datosY = new QVector <QPointF> ();

        QVector <QPointF> * datosFases = new QVector <QPointF> ();
        QVector <QPointF> * datosMag = new QVector <QPointF> ();

        qreal tamFas = boundaries->getTamFas() - 1;


        foreach (auto vector, *boun) {

            QVector<QPointF> * nuevoVector = new QVector<QPointF>  ();

            QVector <QVector <QPointF> * > * nuevoHash = new QVector <QVector <QPointF> *> ();

            for (qint32 i = 0; i <= tamFas; ++i) {
                nuevoHash->append(new QVector <QPointF> ());
            }

            qreal nuevosDatosXMin = std::numeric_limits<qreal>::max(), nuevosDatosXMax = std::numeric_limits<qreal>::lowest();
            qreal nuevosDatosYMin = std::numeric_limits<qreal>::max(), nuevosDatosYMax = std::numeric_limits<qreal>::lowest();

            qreal DatosFasMin = std::numeric_limits<qreal>::max(), DatosFasMax = std::numeric_limits<qreal>::lowest();
            qreal DatosMagMin = std::numeric_limits<qreal>::max(), DatosMagMax = std::numeric_limits<qreal>::lowest();


            foreach (auto p, *vector) {
                maglineal = pow(10.,p.y()/20.);

                QPointF punto (maglineal * cos (p.x() * M_PI / 180.),
                               maglineal * sin (p.x() * M_PI / 180.));

                nuevoVector->append(punto);

                if (punto.x() < nuevosDatosXMin) {
                    nuevosDatosXMin = punto.x();
                }

                if (punto.x() > nuevosDatosXMax) {
                    nuevosDatosXMax = punto.x();
                }

                if (punto.y() < nuevosDatosYMin) {
                    nuevosDatosYMin = punto.y();
                }

                if (punto.y() > nuevosDatosYMax) {
                    nuevosDatosYMax = punto.y();
                }

                if (p.x() < DatosFasMin) {
                    DatosFasMin = p.x();
                }

                if (p.x() > DatosFasMax) {
                    DatosFasMax = p.x();
                }

                if (p.y() < DatosMagMin) {
                    DatosMagMin = p.y();
                }

                if (p.y() > DatosMagMax) {
                    DatosMagMax = p.y();
                }
            }

            qreal a = (nuevosDatosXMax - nuevosDatosXMin);

            foreach (auto p, *nuevoVector) {

                qreal res = (p.x() + std::abs(nuevosDatosXMin)) * tamFas / a;
                nuevoHash->at((qint32) res)->append(p);
            }

            datosX->append(QPointF(nuevosDatosXMin, nuevosDatosXMax));
            datosY->append(QPointF(nuevosDatosYMin, nuevosDatosYMax));

            datosFases->append(QPointF(DatosFasMin, DatosFasMax));
            datosMag->append(QPointF(DatosMagMin, DatosMagMax));

            nuevoHash_inter->append(nuevoHash);

            nuevosBoundariesReun->append(nuevoVector);
        }

        DatosBound * nuevoBoundaries = new DatosBound (boundaries->getBoundaries(), boundaries->getMetaDatosAbierta(),
                                                       boundaries->getMetaDatosArriba(), boundaries->getTamFas(),
                                                       boundaries->getDatosFas(), nuevosBoundariesReun,
                                                       nuevoHash_inter, boundaries->getTamMag(), boundaries->getDatosMag());

        nuevoBoundaries->setDatosFasBoundLin(datosX);
        nuevoBoundaries->setDatosMagBoundLin(datosY);

        nuevoBoundaries->setDatosFasBound(datosFases);
        nuevoBoundaries->setDatosMagBound(datosMag);

        boundariesAux = boundaries;
        this->boundaries = nuevoBoundaries;

    } else {
        deteccionViolacion = &DeteccionViolacionBoundaries::deteccionViolacionCajaNiNi;
        Nyquist = false;

        QVector< QVector<QPointF> * > * boun = boundaries->getBoundariesReun();

        QVector <QPointF> * datosFases = new QVector <QPointF> ();
        QVector <QPointF> * datosMag = new QVector <QPointF> ();

        foreach (auto vector, *boun) {


            qreal DatosFasMin = std::numeric_limits<qreal>::max(), DatosFasMax = std::numeric_limits<qreal>::lowest();
            qreal DatosMagMin = std::numeric_limits<qreal>::max(), DatosMagMax = std::numeric_limits<qreal>::lowest();


            foreach (auto p, *vector) {

                if (p.x() < DatosFasMin) {
                    DatosFasMin = p.x();
                }

                if (p.x() > DatosFasMax) {
                    DatosFasMax = p.x();
                }

                if (p.y() < DatosMagMin) {
                    DatosMagMin = p.y();
                }

                if (p.y() > DatosMagMax) {
                    DatosMagMax = p.y();
                }
            }

            datosFases->append(QPointF(DatosFasMin, DatosFasMax));
            datosMag->append(QPointF(DatosMagMin, DatosMagMax));

        }

        boundaries->setDatosFasBound(datosFases);
        boundaries->setDatosMagBound(datosMag);

    }

   // if (a){
    analisis = &Algorithm_isaac::aceleratedNuevo;
    /*} else {
        analisis = &Algorithm_isaac::aceleratedAntiguo;
    }*/
}


//Función principal del algoritmo

bool Algorithm_isaac::init_algorithm() {

    lista = new ListaOrdenada();

    conversion = new Natura_Interval_extension();

    Tripleta * tripleta;
    Sistema * actual;

    struct FC::return_bisection retur;
    deteccion = new DeteccionViolacionBoundaries();


    plantas_nominales = new QVector <cxsc::complex> ();
    plantas_nominales2 = new QVector <std::complex <qreal>> ();

    foreach (qreal o, *omega) {
        std::complex <qreal> c = planta->getPunto(o);
        plantas_nominales2->append(c);
        plantas_nominales->append(cxsc::complex(c.real(), c.imag()));
    }


    comprobarVariables(controlador);

    tripleta = check_box_feasibility(controlador);

    if (tripleta == nullptr) {
        menerror("El espacio de parámetros inicial del controlador no es válido.", "Loop Shaping");

        delete conversion;
        delete lista;
        delete deteccion;

        return false;
    }

    lista->insertar(tripleta);

    while (true) {

        if (lista->isVacia()) {
            menerror("El espacio de parámetros inicial del controlador no es válido.", "Loop Shaping");

            delete conversion;
            delete lista;
            delete deteccion;

            return false;
        }


        tripleta = lista->recuperar();
        lista->borrarPrimero();

        actual = tripleta->getSistema();

        if (tripleta->flags == feasible || FC::if_less_epsilon(actual, epsilon, omega, conversion, plantas_nominales)) {
            if (tripleta->flags == ambiguous) {
                controlador_retorno = FC::guardarControlador(tripleta->getSistema(), false);
            } else {
                controlador_retorno = FC::guardarControlador(tripleta->getSistema(), true);
            }

            delete conversion;
            delete lista;
            delete deteccion;
            delete tripleta;

            return true;
        }

        delete tripleta;


        //Split blox
        retur = (this->*split_box)(actual);

        if (retur.v1 != nullptr){
            tripleta = check_box_feasibility(retur.v1);
            if (tripleta != nullptr) {
                lista->insertar(tripleta);
            }
        }

        if (retur.v2 != nullptr){
            tripleta = check_box_feasibility(retur.v2);
            if (tripleta != nullptr) {
                lista->insertar(tripleta);
            }
        }
    }
}

//Función que retorna el controlador.

Sistema * Algorithm_isaac::getControlador() {
    return controlador_retorno;
}


//Función que comprueba si la caja actual es feasible, infeasible o ambiguous.

inline Tripleta * Algorithm_isaac::check_box_feasibility(Sistema *controlador) {

    using namespace std;

    struct datos_caja datos;

    flags_box flag_final = feasible;

    QVector <datos_caja> * datosCortesBoundaries = new QVector <datos_caja> ();

    Tripleta * t = new Tripleta();

    cinterval caja;
    bool penalizacion = false;


#if defined(COMPARACION_CAJAS)
    ViewBoundReun * view = new ViewBoundReun();

    view->setDatos(boundaries->getBoundariesReun(), omega);

    view->mostrar_diagrama();

    QVector <qreal> areasCaja;
#endif

    for (qint32 k = 0; k < omega->size(); k++) {

        caja = conversion->get_box(controlador, omega->at(k), plantas_nominales->at(k), Nyquist);


#ifdef VER_ANTES
        ViewBoundReun * view = new ViewBoundReun();

        view->setDatos(boundaries->getBoundariesReun(), omega);

        view->mostrar_diagrama();
#endif

#if defined(COMPARACION_CAJAS) || defined(VER_ANTES)

#ifdef COMPARACION_CAJAS
        areasCaja.append(_double(diam(Re(caja)) * diam(Im(caja))));
#endif

        if (Nyquist){
            view->dibujar_cuadro(QPointF(_double(InfRe(caja)), _double(InfIm(caja))),
                                 QPointF(_double(InfRe(caja)), _double(SupIm(caja))),QPointF(_double(SupRe(caja)), _double(SupIm(caja))), QPointF(_double(SupRe(caja)), _double(InfIm(caja))), k);
        } else {
            view->dibujar_cuadro(QPointF(_double(InfIm(caja)), _double(InfRe(caja))), QPointF(_double(InfIm(caja)), _double(SupRe(caja))),
                                 QPointF(_double(SupIm(caja)), _double(SupRe(caja))), QPointF(_double(SupIm(caja)), _double(InfRe(caja))), k);
        }

#ifdef VER_ANTES
        view->exec();
        delete view;
#endif
#endif

        boundaries->setBox(conversion->getBoxDB());

        datos = (deteccion->*deteccionViolacion)(caja, boundaries, k);

        //////////////////////////////////////////////////////////////////////////
        /*QVector <QPointF> * v = new QVector <QPointF> ();

        cinterval caja = conversion->get_box(controlador,omega->at(k), plantas_nominales->at(k), false);

        v->append(QPointF(InfIm(caja), InfRe(caja)));
        v->append(QPointF(InfIm(caja), SupRe(caja)));
        v->append(QPointF(SupIm(caja), SupRe(caja)));
        v->append(QPointF(SupIm(caja), InfRe(caja)));

        //FC::mostrar_diagramaBox(v, this->omega, boundaries);
        //vectorCajas->append(v);
        vectorareas->append(area(caja));*/
        /////////////////////////////////////////////////////////////////////////////

        //cout << "f " << omega->at(k) << ": " << (datos.flag == 0 ? "Feasible" : datos.flag == 1 ? "Infeasible" : "Ambiguo") << " Comp: " << (datos.completo ? "true" : "false") << endl;

        if (datos.flag == infeasible) {
            delete controlador;
            datosCortesBoundaries->clear();
            return nullptr;
        }

        /*if (omega->at(k) == 2 && (Nyquist ? conversion->getBoxInf() < -180 : SupIm(caja) < -180)) {
            penalizacion = true;
        }*/

        if (datos.flag == ambiguous) {
            flag_final = ambiguous;
        }



        datosCortesBoundaries->append(datos);
    }


    if (flag_final == ambiguous) {
        controlador = (this->*analisis)(controlador, datosCortesBoundaries);
    }


#ifdef COMPARACION_CAJAS

    for (qint32 k = 0; k < omega->size(); k++) {

        caja = conversion->get_box(controlador, omega->at(k), plantas_nominales->at(k), Nyquist);

        qreal a = _double(diam(Re(caja)) * diam(Im(caja)));

        cout << "Área " << omega->at(k) <<": " << areasCaja.at(k) << ", " << a << ": " << areasCaja.at(k) - a << endl;

        /*cout << "k: [" << controlador->getK()->getRango().x() << ", " << controlador->getK()->getRango().y();

        cout << " a: [" << controlador->getNumerador()->at(0)->getRango().x() << ", " << controlador->getNumerador()->at(0)->getRango().y() << "]" ;

        cout << " b: [" << controlador->getNumerador()->at(1)->getRango().x() << ", " << controlador->getNumerador()->at(1)->getRango().y() << "]" ;

        cout << " c: [" << controlador->getNumerador()->at(2)->getNominal() << "]" ;


        cout << " d: [" << controlador->getDenominador()->at(0)->getRango().x() << ", " << controlador->getDenominador()->at(0)->getRango().y() << "]" ;

        cout << " e: [" << controlador->getDenominador()->at(1)->getRango().x() << ", " << controlador->getDenominador()->at(1)->getRango().y() << "]" ;

        cout << " f: [" << controlador->getDenominador()->at(2)->getNominal() << "]" ;

        cout << " g: [" << controlador->getDenominador()->at(3)->getNominal() << "]" << endl;*/



        if (Nyquist){
            view->dibujar_cuadro2(QPointF(_double(InfRe(caja)), _double(InfIm(caja))),
                                  QPointF(_double(InfRe(caja)), _double(SupIm(caja))),QPointF(_double(SupRe(caja)), _double(SupIm(caja))), QPointF(_double(SupRe(caja)), _double(InfIm(caja))), k);
        } else {
            view->dibujar_cuadro2(QPointF(_double(InfIm(caja)), _double(InfRe(caja))), QPointF(_double(InfIm(caja)), _double(SupRe(caja))),
                                  QPointF(_double(SupIm(caja)), _double(SupRe(caja))), QPointF(_double(SupIm(caja)), _double(InfRe(caja))), k);
        }

    }

    view->exec();

    delete view;

#endif

    t->setSistema(controlador);


#ifdef BE
    t->setIndex(-(20 * nFeasible) + 10 * t->getPorcentajeFeasible());
#else
    if (penalizacion){
        t->setIndex(controlador->getK()->getRango().x() + 100);
    } else {
        t->setIndex(controlador->getK()->getRango().x());
    }
#endif



    ///////////////////////////////////////////////////////////////////

    /*ViewBoundReun * view = FC::mostrar_diagrama(vectorCajas, this->omega, boundaries);
    vectorCajas->clear();

    vectorCajas = new QVector <QVector <QPointF> *> ();

    for (qint32 k = 0; k < omega->size(); k++) {
        QVector <QPointF> * v = new QVector <QPointF> ();

        cinterval caja = conversion->get_box(controlador,omega->at(k), plantas_nominales->at(k), true);

        v->append(QPointF(InfIm(caja), InfRe(caja)));
        v->append(QPointF(InfIm(caja), SupRe(caja)));
        v->append(QPointF(SupIm(caja), SupRe(caja)));
        v->append(QPointF(SupIm(caja), InfRe(caja)));

        //FC::mostrar_diagramaBox(v, this->omega, boundaries);
        vectorCajas->append(v);
        //cout << vectorareas->at(k) << ";" << area(caja) << ";" << k << ";" << endl;
    }

    FC::mostrar_diagrama2(vectorCajas, this->omega, nuevoBoundaries, view);
    vectorCajas->clear();
    vectorareas->clear();*/

    /////////////////////////////////////////////////////////////////////

    t->setFlags(flag_final);
    datosCortesBoundaries->clear();

    return t;
}


//Función que recorta la caja.
inline Sistema * Algorithm_isaac::aceleratedNuevo(Sistema * v, QVector <datos_caja> * datosCortesBoundaries) {
    QVector <Var *> * denominador = v->getDenominador();
    QVector <Var *> * numerador = v->getNumerador();
    QPointF k = v->getK()->getRango();
    QPointF kNuevo = v->getK()->getRango();

#if defined(REC_INTER) || defined(GANANCIA)
    bool kMagUnionArriba = false, kMagUnionAbajo = false, kIntersecionX = false, kIntersecionY = false;
    qreal kNuevoIntersecionX = kNuevo.y() + 1;
    qreal kNuevoIntersecionY = kNuevo.x() - 1;
#endif

    //Creamos los numeradores y denominadores necesarios
    QVector <qreal> * numeradorSup = new QVector <qreal> ();
    QVector <qreal> * numeradorInf = new QVector <qreal> ();
    QVector <qreal> * numeradorSupNuevo = new QVector <qreal> ();
    QVector <qreal> * numeradorInfNuevo = new QVector <qreal> ();

#ifdef REC_INTER
    QVector <bool> * numeradorUnionX = new QVector <bool> ();
    QVector <bool> * numeradorUnionY = new QVector <bool> ();
    QVector <bool> * numeradorInterseccionX = new QVector <bool> ();
    QVector <bool> * numeradorInterseccionY = new QVector <bool> ();
    QVector <qreal> * numeradorNuevoInterseccionX = new QVector <qreal> ();
    QVector <qreal> * numeradorNuevoInterseccionY = new QVector <qreal> ();
#endif

    foreach (Var * var, *numerador) {
        numeradorInf->append(var->getRango().x());
        numeradorSup->append(var->getRango().y());
        numeradorInfNuevo->append(var->getRango().x());
        numeradorSupNuevo->append(var->getRango().y());

#ifdef REC_INTER
        numeradorUnionX->append(false);
        numeradorUnionY->append(false);
        numeradorInterseccionX->append(false);
        numeradorInterseccionY->append(false);
        numeradorNuevoInterseccionX->append(var->getRango().y() + 1);
        numeradorNuevoInterseccionY->append(var->getRango().x() - 1);
#endif
    }

    QVector <qreal> * denominadorInf = new QVector <qreal> ();
    QVector <qreal> * denominadorSup = new QVector <qreal> ();
    QVector <qreal> * denominadorInfNuevo = new QVector <qreal> ();
    QVector <qreal> * denominadorSupNuevo = new QVector <qreal> ();
#ifdef REC_INTER
    QVector <bool> * denominadorUnionX = new QVector <bool> ();
    QVector <bool> * denominadorUnionY = new QVector <bool> ();
    QVector <bool> * denominadorInterseccionX = new QVector <bool> ();
    QVector <bool> * denominadorInterseccionY = new QVector <bool> ();
    QVector <qreal> * denominadorNuevoInterseccionX = new QVector <qreal> ();
    QVector <qreal> * denominadorNuevoInterseccionY = new QVector <qreal> ();
#endif

    foreach (Var * var, *denominador) {
        denominadorInf->append(var->getRango().x());
        denominadorSup->append(var->getRango().y());
        denominadorInfNuevo->append(var->getRango().x());
        denominadorSupNuevo->append(var->getRango().y());
#ifdef REC_INTER
        denominadorUnionX->append(false);
        denominadorUnionY->append(false);
        denominadorInterseccionX->append(false);
        denominadorInterseccionY->append(false);
        denominadorNuevoInterseccionX->append(var->getRango().y() + 1);
        denominadorNuevoInterseccionY->append(var->getRango().x() - 1);
#endif
    }

    qreal nuevoMinKReal, n, nuevoMinNume, nuevoMaxDeno, o, nuevoMaxKReal, nuevoMaxNume, nuevoMinDeno, cortesMin, cortesMax, cortesMinImag, cortesMaxImag;
    std::complex <qreal> plantaNominal;
    for (qint32 i = 0; i < omega->size(); i++) {

        if (datosCortesBoundaries->at(i).flag == ambiguous) {
            o = omega->at(i);
            plantaNominal = plantas_nominales2->at(i);
            cortesMin = datosCortesBoundaries->at(i).minimosMaximos->at(0);
            cortesMax = datosCortesBoundaries->at(i).minimosMaximos->at(1);
            cortesMinImag = datosCortesBoundaries->at(i).minimosMaximos->at(2);
            cortesMaxImag = datosCortesBoundaries->at(i).minimosMaximos->at(3);

            //Análisis de la ganancia
//#ifdef REC_MAG
            if (datosCortesBoundaries->at(i).recAbajo){

                if (datosCortesBoundaries->at(i).uniAbajo){
//#ifdef REC_UNION
                    nuevoMinKReal = cortesMin / abs(v->getPunto(numeradorSup, denominadorInf, 1, 0, o) * plantaNominal);

                    if (nuevoMinKReal > kNuevo.x() && nuevoMinKReal < kNuevo.y()) {
                        kNuevo.setX(nuevoMinKReal);
#if defined(REC_INTER) || defined(GANANCIA)
                        kMagUnionAbajo = true;
#endif
                    }
//#endif
                } else {
#if defined(REC_INTER) || defined(GANANCIA)
                    if (!kMagUnionAbajo){
                        nuevoMinKReal = cortesMin / abs(v->getPunto(numeradorSup, denominadorInf, 1, 0, o) * plantaNominal);

                        if (nuevoMinKReal > kNuevo.x() && nuevoMinKReal < kNuevo.y() && nuevoMinKReal < kNuevoIntersecionX) {
                            kNuevoIntersecionX = nuevoMinKReal;
                            kIntersecionX = true;
                        }
                    }
#endif
                }

            }

            if (datosCortesBoundaries->at(i).recArriba){
                if (datosCortesBoundaries->at(i).uniArriba){
//#ifdef REC_UNION
                    nuevoMaxKReal = cortesMax / abs(v->getPunto(numeradorInf, denominadorSup, 1, 0, o) * plantaNominal);

                    if (nuevoMaxKReal > kNuevo.x() && nuevoMaxKReal < kNuevo.y()) {
                        kNuevo.setY(nuevoMaxKReal);
#if defined(REC_INTER) || defined(GANANCIA)
                        kMagUnionArriba = true;
#endif
                    }

//#endif
                } else {

#if defined(REC_INTER) || defined(GANANCIA)
                    if (!kMagUnionArriba){
                        nuevoMinKReal = cortesMin / abs(v->getPunto(numeradorSup, denominadorInf, 1, 0, o) * plantaNominal);

                        if (nuevoMinKReal > kNuevo.x() && nuevoMinKReal < kNuevo.y() && nuevoMinKReal < kNuevoIntersecionX) {
                            kNuevoIntersecionX = nuevoMinKReal;
                            kIntersecionX = true;
                        }
                    }
#endif
                }

            }
//#endif

#ifndef sachin

            //Numerador
            if (isVariableNume){
                for (qint32 j = 0; j < numerador->size(); j++) {
                    if (numerador->at(j)->isVariable()){

                        n = numeradorSup->at(j);
                        numeradorSup->remove(j);

//#ifdef REC_MAG
                        if (datosCortesBoundaries->at(i).recAbajo){
                            if (datosCortesBoundaries->at(i).uniAbajo){

//#ifdef REC_UNION
                                nuevoMinNume = sqrt( pow((cortesMin * abs (v->getPuntoDeno(denominadorInf, o))) /
                                                         (k.y() *  abs (v->getPuntoNume(numeradorSup, o) * plantaNominal)), 2) - pow(o, 2));

                                if (nuevoMinNume > numeradorInfNuevo->at(j) && nuevoMinNume < numeradorSupNuevo->at(j)) {
                                    numeradorInfNuevo->replace(j, nuevoMinNume);
#ifdef REC_INTER
                                    numeradorUnionX->replace(j, true);
#endif
                                }
//#endif
                            } else {
#if defined(REC_INTER) && defined(REC_MAG)
                                if (!numeradorUnionX->at(j)){
                                    nuevoMinNume = sqrt( pow((cortesMin * abs (v->getPuntoDeno(denominadorInf, o))) /
                                                             (k.y() *  abs (v->getPuntoNume(numeradorSup, o) * plantaNominal)), 2) - pow(o, 2));

                                    if (nuevoMinNume > numeradorInfNuevo->at(j) && nuevoMinNume < numeradorSupNuevo->at(j) && nuevoMinNume < numeradorNuevoInterseccionX->at(j)) {
                                        numeradorNuevoInterseccionX->replace(j, nuevoMinNume);
                                        numeradorInterseccionX->replace(j, true);
                                    }
                                }
#endif
                            }
                        }
//#endif

#ifdef REC_FASE
                        if (datosCortesBoundaries->at(i).recDerecha){
                            if (datosCortesBoundaries->at(i).uniDerecha){
#if defined(REC_UNION)
                                nuevoMaxNume = tan(cortesMaxImag - std::arg (v->getPuntoNume(numeradorSup, o) + std::arg (v->getPuntoDeno(denominadorInf, o))) - std::arg (plantaNominal)) * o;

                                if (nuevoMaxNume > numeradorInfNuevo->at(j) && nuevoMaxNume < numeradorSupNuevo->at(j)) {
                                    numeradorSupNuevo->replace(j, nuevoMaxNume);
#ifdef REC_INTER
                                    numeradorUnionY->replace(j, true);
#endif
                                }
#endif
                            } else {
#if defined(REC_INTER)
                                if (!numeradorUnionY->at(j)){
                                    nuevoMaxNume = tan(cortesMaxImag - std::arg (v->getPuntoNume(numeradorSup, o) + std::arg (v->getPuntoDeno(denominadorInf, o))) - std::arg (plantaNominal)) * o;
                                    if (nuevoMaxNume > numeradorInfNuevo->at(j) && nuevoMaxNume < numeradorSupNuevo->at(j) && nuevoMaxNume > numeradorNuevoInterseccionY->at(j)) {
                                        numeradorNuevoInterseccionY->replace(j, nuevoMaxNume);
                                        numeradorInterseccionY->replace(j, true);
                                    }
                                }
#endif
                            }
                        }

#endif

                        numeradorSup->insert(j, n);



                        n = numeradorInf->at(j);
                        numeradorInf->remove(j);

//#ifdef REC_MAG
                        if (datosCortesBoundaries->at(i).recArriba){
                            if (datosCortesBoundaries->at(i).uniArriba){
//#if defined(REC_UNION)
                                nuevoMaxNume = sqrt( pow((cortesMax * abs (v->getPuntoDeno(denominadorSup, o))) /
                                                         (k.x() *  abs (v->getPuntoNume(numeradorInf, o) * plantaNominal)), 2) - pow(o, 2));

                                if (nuevoMaxNume > numeradorInfNuevo->at(j) && nuevoMaxNume < numeradorSupNuevo->at(j)) {
                                    numeradorSupNuevo->replace(j, nuevoMaxNume);
#ifdef REC_INTER
                                    numeradorUnionY->replace(j, true);
#endif
                                }
//#endif
                            } else {
#if defined(REC_INTER) && defined(REC_MAG)
                                if (!numeradorUnionY->at(j)){
                                    nuevoMaxNume = sqrt( pow((cortesMax * abs (v->getPuntoDeno(denominadorSup, o))) /
                                                             (k.x() *  abs (v->getPuntoNume(numeradorInf, o) * plantaNominal)), 2) - pow(o, 2));

                                    if (nuevoMaxNume > numeradorInfNuevo->at(j) && nuevoMaxNume < numeradorSupNuevo->at(j) && nuevoMaxNume > numeradorNuevoInterseccionY->at(j)) {
                                        numeradorNuevoInterseccionY->replace(j, nuevoMaxNume);
                                        numeradorInterseccionY->replace(j, true);
                                    }
                                }
#endif
                            }
                        }
//#endif

#ifdef REC_FASE
                        if (datosCortesBoundaries->at(i).recIzquierda){
                            if (datosCortesBoundaries->at(i).uniIzquierda){
#if defined(REC_UNION)
                                nuevoMinNume = tan(cortesMinImag - std::arg (v->getPuntoNume(numeradorInf, o) + std::arg (v->getPuntoDeno(denominadorSup, o))) - std::arg (plantaNominal)) * o;

                                if (nuevoMinNume > numeradorInfNuevo->at(j) && nuevoMinNume < numeradorSupNuevo->at(j)) {
                                    numeradorInfNuevo->replace(j, nuevoMinNume);
#ifdef REC_INTER
                                    numeradorUnionX->replace(j, true);
#endif
                                }
#endif
                            } else {

#if defined(REC_INTER)
                                if (!numeradorUnionX->at(j)){
                                    nuevoMinNume = tan(cortesMinImag - std::arg (v->getPuntoNume(numeradorInf, o) + std::arg (v->getPuntoDeno(denominadorSup, o))) - std::arg (plantaNominal)) * o;

                                    if (nuevoMinNume > numeradorInfNuevo->at(j) && nuevoMinNume < numeradorSupNuevo->at(j) && nuevoMinNume < numeradorNuevoInterseccionX->at(j)) {
                                        numeradorNuevoInterseccionX->replace(j, nuevoMinNume);
                                        numeradorInterseccionX->replace(j, true);
                                    }
                                }
#endif
                            }
                        }
#endif

                        numeradorInf->insert(j, n);

                    }
                }
            }


            if (isVariableDeno){
                for (qint32 j = 0; j < denominador->size(); j++) {
                    if (denominador->at(j)->isVariable()){

                        n = denominadorInf->at(j);
                        denominadorInf->remove(j);

//#ifdef REC_MAG
                        if (datosCortesBoundaries->at(i).recAbajo){
                            if (datosCortesBoundaries->at(i).uniAbajo){
//#if defined(REC_UNION)

                                nuevoMaxDeno = sqrt(pow((k.y() * abs (v->getPuntoNume(numeradorSup, o) * plantaNominal)) /
                                                        (cortesMin * abs(v->getPuntoDeno(denominadorInf, o))), 2) - pow(o, 2));

                                if (nuevoMaxDeno < denominadorSupNuevo->at(j) && nuevoMaxDeno > denominadorInfNuevo->at(j)) {
                                    denominadorSupNuevo->replace(j, nuevoMaxDeno);
#ifdef REC_INTER
                                    denominadorUnionY->replace(j, true);
#endif
                                }
//#endif
                            } else {
#if defined(REC_INTER) && defined(REC_MAG)
                                if (!denominadorUnionY->at(j)){
                                    nuevoMaxDeno = sqrt(pow((k.y() * abs (v->getPuntoNume(numeradorSup, o) * plantaNominal)) /
                                                            (cortesMin * abs(v->getPuntoDeno(denominadorInf, o))), 2) - pow(o, 2));

                                    if (nuevoMaxDeno > denominadorInfNuevo->at(j) && nuevoMaxDeno < denominadorSupNuevo->at(j) && nuevoMaxDeno > denominadorNuevoInterseccionY->at(j)) {
                                        denominadorNuevoInterseccionY->replace(j, nuevoMaxDeno);
                                        denominadorInterseccionY->replace(j, true);
                                    }
                                }
#endif
                            }
                        }
//#endif

#ifdef REC_FASE
                        if (datosCortesBoundaries->at(i).recIzquierda){
                            if (datosCortesBoundaries->at(i).uniIzquierda){
#if defined(REC_UNION)
                                nuevoMinDeno = tan(-cortesMaxImag + std::arg (v->getPuntoNume(numeradorInf, o) - std::arg (v->getPuntoDeno(denominadorSup, o))) + std::arg (plantaNominal)) * o;

                                if (nuevoMinDeno > denominadorInfNuevo->at(j) && nuevoMinDeno < denominadorSupNuevo->at(j)) {
                                    denominadorInfNuevo->replace(j, nuevoMinDeno);
#ifdef REC_INTER
                                    denominadorUnionX->replace(j, true);
#endif
                                }
#endif
                            } else {
#if defined(REC_INTER)
                                if (!denominadorUnionX->at(j)){
                                    nuevoMinDeno = tan(-cortesMaxImag + std::arg (v->getPuntoNume(numeradorInf, o) - std::arg (v->getPuntoDeno(denominadorSup, o))) + std::arg (plantaNominal)) * o;

                                    if (nuevoMinDeno > denominadorInfNuevo->at(j) && nuevoMinDeno < denominadorSupNuevo->at(j) && nuevoMinDeno < denominadorNuevoInterseccionX->at(j)) {
                                        denominadorNuevoInterseccionX->replace(j, nuevoMinDeno);
                                        denominadorInterseccionX->replace(j, true);
                                    }
                                }
#endif
                            }
                        }
#endif

                        denominadorInf->insert(j, n);

                        n = denominadorSup->at(j);
                        denominadorSup->remove(j);
//#ifdef REC_MAG
                        if (datosCortesBoundaries->at(i).recArriba){
                            if (datosCortesBoundaries->at(i).uniArriba){
//#if defined(REC_UNION)
                                nuevoMinDeno = sqrt(pow((k.x() * abs (v->getPuntoNume(numeradorInf, o) * plantaNominal)) /
                                                        (cortesMax * abs(v->getPuntoDeno(denominadorSup, o))), 2) - pow(o, 2));
                                if (nuevoMinDeno < denominadorSupNuevo->at(j) && nuevoMinDeno > denominadorInfNuevo->at(j)) {
                                    denominadorInfNuevo->replace(j, nuevoMinDeno);
#ifdef REC_INTER
                                    denominadorUnionX->replace(j, true);
#endif
                                }
//#endif
                            } else {
#if defined(REC_INTER) && defined(REC_MAG)
                                if (!denominadorUnionX->at(j)){
                                    nuevoMinDeno = sqrt(pow((k.x() * abs (v->getPuntoNume(numeradorInf, o) * plantaNominal)) /
                                                            (cortesMax * abs(v->getPuntoDeno(denominadorSup, o))), 2) - pow(o, 2));

                                    if (nuevoMinDeno < denominadorSupNuevo->at(j) && nuevoMinDeno > denominadorInfNuevo->at(j) && nuevoMinDeno < denominadorNuevoInterseccionX->at(j)) {
                                        denominadorNuevoInterseccionX->replace(j, nuevoMinDeno);
                                        denominadorInterseccionX->replace(j, true);
                                    }
                                }
#endif
                            }
                        }
//#endif

#ifdef REC_FASE
                        if (datosCortesBoundaries->at(i).recDerecha){
                            if (datosCortesBoundaries->at(i).uniDerecha){
#if defined(REC_UNION)
                                nuevoMaxDeno = tan(-cortesMinImag + std::arg (v->getPuntoNume(numeradorInf, o) - std::arg (v->getPuntoDeno(denominadorSup, o))) + std::arg (plantaNominal)) * o;

                                if (nuevoMaxDeno > denominadorInfNuevo->at(j) && nuevoMaxDeno < denominadorSupNuevo->at(j)) {
                                    denominadorSupNuevo->replace(j, nuevoMaxDeno);
#ifdef REC_INTER
                                    denominadorUnionY->replace(j, true);
#endif
                                }
#endif
                            } else {

#if defined(REC_INTER)
                                if (!denominadorUnionY->at(j)){
                                    nuevoMaxDeno = tan(-cortesMinImag + std::arg (v->getPuntoNume(numeradorInf, o) - std::arg (v->getPuntoDeno(denominadorSup, o))) + std::arg (plantaNominal)) * o;

                                    if (nuevoMaxDeno > denominadorInfNuevo->at(j) && nuevoMaxDeno < denominadorSupNuevo->at(j) && nuevoMaxDeno > denominadorNuevoInterseccionY->at(j)) {
                                        denominadorNuevoInterseccionY->replace(j, nuevoMaxDeno);
                                        denominadorInterseccionY->replace(j, true);
                                    }
                                }
#endif
                            }
                        }
#endif

                        denominadorSup->insert(j, n);

                    }
                }
            }
#endif

        }
    }


    QVector <Var *> * numerador_nuevo;

    //if (numeradorInterseccionX || numeradorInterseccionY || numeradorUnionX || numeradorUnionY){
        numerador_nuevo = new QVector <Var *> ();

        for (qint32 i = 0; i < numerador->size(); i++){

            Var * var_nume_antiguo = numerador->at(i);
            Var * var_nume_nuevo;

            if (var_nume_antiguo->isVariable()){

#ifdef REC_INTER
                if (numeradorInterseccionX->at(i) && numeradorNuevoInterseccionX->at(i) > numeradorInfNuevo->at(i)){
                    numeradorInfNuevo->replace(i, numeradorNuevoInterseccionX->at(i));
                }

                if (numeradorInterseccionY->at(i) && numeradorNuevoInterseccionY->at(i) < numeradorSupNuevo->at(i)){
                    numeradorSupNuevo->replace(i, numeradorNuevoInterseccionY->at(i));
                }
#endif
                var_nume_nuevo = new Var("", QPointF(numeradorInfNuevo->at(i), numeradorSupNuevo->at(i)), 0);
            } else {
                var_nume_nuevo = new Var(var_nume_antiguo->getNominal());
            }

            numerador_nuevo->append(var_nume_nuevo);
        }
    /*} else {
        numerador_nuevo = numerador;
    }*/

    QVector <Var *> * denominador_nuevo;

    //if (denominadorInterseccionX || denominadorInterseccionY || denominadorUnionX || denominadorUnionY){
        denominador_nuevo = new QVector <Var *> ();
        for (qint32 i = 0; i < denominador->size(); i++){

            Var * var_deno_antiguo = denominador->at(i);
            Var * var_deno_nuevo;

            if (var_deno_antiguo->isVariable()){

#ifdef REC_INTER
                if (denominadorInterseccionX->at(i) && denominadorNuevoInterseccionX->at(i) > denominadorInfNuevo->at(i)){
                    denominadorInfNuevo->replace(i, denominadorNuevoInterseccionX->at(i));
                }

                if (denominadorInterseccionY->at(i) && denominadorNuevoInterseccionY->at(i) < denominadorSupNuevo->at(i)){
                    denominadorSupNuevo->replace(i, denominadorNuevoInterseccionY->at(i));
                }
#endif

                var_deno_nuevo = new Var("", QPointF(denominadorInfNuevo->at(i), denominadorSupNuevo->at(i)), 0);
            } else {
                var_deno_nuevo = new Var(var_deno_antiguo->getNominal());
            }

            denominador_nuevo->append(var_deno_nuevo);
        }

    /*} else {
        denominador_nuevo = denominador;
    }*/

#if defined(REC_INTER) || defined(GANANCIA)
    if (kIntersecionX){
        if (kNuevoIntersecionX > kNuevo.x()){
            kNuevo.setX(kNuevoIntersecionX);
        }
    }

    if (kIntersecionY){
        if (kNuevoIntersecionY < kNuevo.y()){
            kNuevo.setY(kNuevoIntersecionY);
        }
    }
#endif

    Sistema * nuevo_sistema = v->invoke(v->getNombre(), numerador_nuevo, denominador_nuevo, new Var("kv", kNuevo, 0), new Var (0.0));
    delete v;


    numeradorSup->clear();
    numeradorInf->clear();
    numeradorInfNuevo->clear();
    numeradorSupNuevo->clear();
#ifdef REC_INTER
    numeradorUnionX->clear();
    numeradorUnionY->clear();
    numeradorInterseccionX->clear();
    numeradorInterseccionY->clear();
    numeradorNuevoInterseccionX->clear();
    numeradorNuevoInterseccionY->clear();
#endif

    denominadorInf->clear();
    denominadorSup->clear();
    denominadorInfNuevo->clear();
    denominadorSupNuevo->clear();
#ifdef REC_INTER
    denominadorUnionX->clear();
    denominadorUnionY->clear();
    denominadorInterseccionX->clear();
    denominadorInterseccionY->clear();
    denominadorNuevoInterseccionX->clear();
    denominadorNuevoInterseccionY->clear();
#endif

    return nuevo_sistema;

}



inline Sistema * Algorithm_isaac::aceleratedAntiguo(Sistema * v, QVector <datos_caja> * datosCortesBoundaries) {

    QVector <Var *> * denominador = v->getDenominador();
    QVector <Var *> * numerador = v->getNumerador();
    QPointF k = v->getK()->getRango();
    QPointF kNuevo = v->getK()->getRango();
#ifdef REC_INTER
    bool kMagUnionArriba = false, kMagUnionAbajo = false, kIntersecionX = false, kIntersecionY = false;
    qreal kNuevoIntersecionX = kNuevo.y() + 1;
    qreal kNuevoIntersecionY = kNuevo.x() - 1;
#endif

    //Creamos los numeradores y denominadores necesarios
    QVector <qreal> * numeradorSup = new QVector <qreal> ();
    QVector <qreal> * numeradorInf = new QVector <qreal> ();
    QVector <qreal> * numeradorSupNuevo = new QVector <qreal> ();
    QVector <qreal> * numeradorInfNuevo = new QVector <qreal> ();
    QVector <bool> * numeradorUnionX = new QVector <bool> ();
    QVector <bool> * numeradorUnionY = new QVector <bool> ();

#ifdef REC_INTER
    QVector <bool> * numeradorInterseccionX = new QVector <bool> ();
    QVector <bool> * numeradorInterseccionY = new QVector <bool> ();
    QVector <qreal> * numeradorNuevoInterseccionX = new QVector <qreal> ();
    QVector <qreal> * numeradorNuevoInterseccionY = new QVector <qreal> ();
#endif

    foreach (Var * var, *numerador) {
        numeradorInf->append(var->getRango().x());
        numeradorSup->append(var->getRango().y());
        numeradorInfNuevo->append(var->getRango().x());
        numeradorSupNuevo->append(var->getRango().y());
        numeradorUnionX->append(false);
        numeradorUnionY->append(false);

#ifdef REC_INTER
        numeradorInterseccionX->append(false);
        numeradorInterseccionY->append(false);
        numeradorNuevoInterseccionX->append(var->getRango().y() + 1);
        numeradorNuevoInterseccionY->append(var->getRango().x() - 1);
#endif
    }

    QVector <qreal> * denominadorInf = new QVector <qreal> ();
    QVector <qreal> * denominadorSup = new QVector <qreal> ();
    QVector <qreal> * denominadorInfNuevo = new QVector <qreal> ();
    QVector <qreal> * denominadorSupNuevo = new QVector <qreal> ();
    QVector <bool> * denominadorUnionX = new QVector <bool> ();
    QVector <bool> * denominadorUnionY = new QVector <bool> ();
#ifdef REC_INTER
    QVector <bool> * denominadorInterseccionX = new QVector <bool> ();
    QVector <bool> * denominadorInterseccionY = new QVector <bool> ();
    QVector <qreal> * denominadorNuevoInterseccionX = new QVector <qreal> ();
    QVector <qreal> * denominadorNuevoInterseccionY = new QVector <qreal> ();
#endif

    foreach (Var * var, *denominador) {
        denominadorInf->append(var->getRango().x());
        denominadorSup->append(var->getRango().y());
        denominadorInfNuevo->append(var->getRango().x());
        denominadorSupNuevo->append(var->getRango().y());
        denominadorUnionX->append(false);
        denominadorUnionY->append(false);
#ifdef REC_INTER
        denominadorInterseccionX->append(false);
        denominadorInterseccionY->append(false);
        denominadorNuevoInterseccionX->append(var->getRango().y() + 1);
        denominadorNuevoInterseccionY->append(var->getRango().x() - 1);
#endif
    }

    qreal nuevoMinKReal, n, nuevoMinNume, nuevoMaxDeno, o, nuevoMaxKReal, nuevoMaxNume, nuevoMinDeno, cortesMin, cortesMax, cortesMinImag, cortesMaxImag;
    std::complex <qreal> plantaNominal;
    for (qint32 i = 0; i < omega->size(); i++) {

        if (datosCortesBoundaries->at(i).flag == ambiguous) {

            if (!datosCortesBoundaries->at(i).completo) {
                o = omega->at(i);
                plantaNominal = plantas_nominales2->at(i);
                cortesMin = datosCortesBoundaries->at(i).minimosMaximos->at(0);
                cortesMax = datosCortesBoundaries->at(i).minimosMaximos->at(1);
                cortesMinImag = datosCortesBoundaries->at(i).minimosMaximos->at(2);
                cortesMaxImag = datosCortesBoundaries->at(i).minimosMaximos->at(3);

                //Análisis de la ganancia

#ifdef REC_MAG

                if (!datosCortesBoundaries->at(i).uniArriba){
#ifdef REC_UNION

                    if (datosCortesBoundaries->at(i).recAbajo){

                        nuevoMinKReal = cortesMin / abs(v->getPunto(numeradorSup, denominadorInf, 1, 0, o) * plantaNominal);

                        if (nuevoMinKReal > kNuevo.x() && nuevoMinKReal < kNuevo.y()) {
                            kNuevo.setX(nuevoMinKReal);
#ifdef REC_INTER
                            kMagUnionAbajo = true;
#endif

                        }
                    }
#endif
#ifdef REC_INTER
                    if (!kMagUnionAbajo && datosCortesBoundaries->at(i).recArriba){
                        nuevoMaxKReal = cortesMax / abs(v->getPunto(numeradorInf, denominadorSup, 1, 0, o) * plantaNominal);

                        if (nuevoMaxKReal > kNuevo.x() && nuevoMaxKReal < kNuevo.y() && nuevoMaxKReal > kNuevoIntersecionY) {
                            kNuevoIntersecionY = nuevoMaxKReal;
                            kIntersecionY = true;
                        }
                    }
#endif
                } else {
#ifdef REC_UNION
                    if (datosCortesBoundaries->at(i).recArriba){
                        nuevoMaxKReal = cortesMax / abs(v->getPunto(numeradorInf, denominadorSup, 1, 0, o) * plantaNominal);

                        if (nuevoMaxKReal > kNuevo.x() && nuevoMaxKReal < kNuevo.y()) {
                            kNuevo.setY(nuevoMaxKReal);
#ifdef REC_INTER
                            kMagUnionArriba = true;
#endif

                        }
                    }
#endif

#ifdef REC_INTER
                    if (!kMagUnionArriba && datosCortesBoundaries->at(i).recAbajo){
                        nuevoMinKReal = cortesMin / abs(v->getPunto(numeradorSup, denominadorInf, 1, 0, o) * plantaNominal);

                        if (nuevoMinKReal > kNuevo.x() && nuevoMinKReal < kNuevo.y() && nuevoMinKReal < kNuevoIntersecionX) {
                            kNuevoIntersecionX = nuevoMinKReal;
                            kIntersecionX = true;
                        }
                    }
#endif
                }

#endif

                //Numerador
                if (isVariableNume){
                    for (qint32 j = 0; j < numerador->size(); j++) {
                        if (numerador->at(j)->isVariable()){
                            if (!datosCortesBoundaries->at(i).uniArriba){
                                //Mag

#if (defined(REC_UNION) || defined(REC_INTER)) && (defined(REC_FASE) || defined(REC_MAG))
                                n = numeradorSup->at(j);

                                numeradorSup->remove(j);

#if defined(REC_UNION) && defined(REC_MAG)

                                if (datosCortesBoundaries->at(i).recAbajo){
                                    nuevoMinNume = sqrt( pow((cortesMin * abs (v->getPuntoDeno(denominadorInf, o))) /
                                                             (k.y() *  abs (v->getPuntoNume(numeradorSup, o) * plantaNominal)), 2) - pow(o, 2));

                                    if (nuevoMinNume > numeradorInfNuevo->at(j) && nuevoMinNume < numeradorSupNuevo->at(j)) {
                                        numeradorInfNuevo->replace(j, nuevoMinNume);
                                        numeradorUnionX->replace(j, true);
                                    }
                                }
#endif
#if defined(REC_INTER) && defined(REC_FASE)
                                if (!numeradorUnionY->at(j) && datosCortesBoundaries->at(i).recDerecha){
                                    nuevoMaxNume = tan(cortesMaxImag - std::arg (v->getPuntoNume(numeradorSup, o) + std::arg (v->getPuntoDeno(denominadorInf, o))) - std::arg (plantaNominal)) * o;
                                    if (nuevoMaxNume > numeradorInfNuevo->at(j) && nuevoMaxNume < numeradorSupNuevo->at(j) && nuevoMaxNume > numeradorNuevoInterseccionY->at(j)) {
                                        numeradorNuevoInterseccionY->replace(j, nuevoMaxNume);
                                        numeradorInterseccionY->replace(j, true);
                                    }
                                }
#endif

                                numeradorSup->insert(j, n);


                                //fase
                                n = numeradorInf->at(j);

                                numeradorInf->remove(j);

#if defined(REC_UNION) && defined(REC_FASE)
                                if (datosCortesBoundaries->at(i).recIzquierda){
                                    nuevoMinNume = tan(cortesMinImag - std::arg (v->getPuntoNume(numeradorInf, o) + std::arg (v->getPuntoDeno(denominadorSup, o))) - std::arg (plantaNominal)) * o;

                                    if (nuevoMinNume > numeradorInfNuevo->at(j) && nuevoMinNume < numeradorSupNuevo->at(j)) {
                                        numeradorInfNuevo->replace(j, nuevoMinNume);
                                        numeradorUnionX->replace(j, true);
                                    }
                                }

#endif

#if defined(REC_INTER) && defined(REC_MAG)
                                if (!numeradorUnionY->at(j) && datosCortesBoundaries->at(i).recDerecha){
                                    nuevoMaxNume = sqrt( pow((cortesMax * abs (v->getPuntoDeno(denominadorSup, o))) /
                                                             (k.x() *  abs (v->getPuntoNume(numeradorInf, o) * plantaNominal)), 2) - pow(o, 2));

                                    if (nuevoMaxNume > numeradorInfNuevo->at(j) && nuevoMaxNume < numeradorSupNuevo->at(j) && nuevoMaxNume > numeradorNuevoInterseccionY->at(j)) {
                                        numeradorNuevoInterseccionY->replace(j, nuevoMaxNume);
                                        numeradorInterseccionY->replace(j, true);
                                    }
                                }
#endif
                                numeradorInf->insert(j, n);
#endif

                            } else {
                                //Mag
#if (defined(REC_UNION) || defined(REC_INTER)) && (defined(REC_FASE) || defined(REC_MAG))

                                n = numeradorInf->at(j);

                                numeradorInf->remove(j);
#if defined(REC_UNION) && defined(REC_MAG)

                                if (datosCortesBoundaries->at(i).recArriba){
                                    nuevoMaxNume = sqrt( pow((cortesMax * abs (v->getPuntoDeno(denominadorSup, o))) /
                                                             (k.x() *  abs (v->getPuntoNume(numeradorInf, o) * plantaNominal)), 2) - pow(o, 2));

                                    if (nuevoMaxNume > numeradorInfNuevo->at(j) && nuevoMaxNume < numeradorSupNuevo->at(j)) {
                                        numeradorSupNuevo->replace(j, nuevoMaxNume);
                                        numeradorUnionY->replace(j, true);
                                    }
                                }

#endif

#if defined(REC_INTER) && defined(REC_FASE)
                                if (!numeradorUnionX->at(j) && datosCortesBoundaries->at(i).recIzquierda){
                                    nuevoMinNume = tan(cortesMinImag - std::arg (v->getPuntoNume(numeradorInf, o) + std::arg (v->getPuntoDeno(denominadorSup, o))) - std::arg (plantaNominal)) * o;

                                    if (nuevoMinNume > numeradorInfNuevo->at(j) && nuevoMinNume < numeradorSupNuevo->at(j) && nuevoMinNume < numeradorNuevoInterseccionX->at(j)) {
                                        numeradorNuevoInterseccionX->replace(j, nuevoMinNume);
                                        numeradorInterseccionX->replace(j, true);
                                    }
                                }
#endif
                                numeradorInf->insert(j, n);

                                //fase

                                n = numeradorSup->at(j);

                                numeradorSup->remove(j);

#if defined(REC_UNION) && defined(REC_FASE)

                                if (datosCortesBoundaries->at(i).recDerecha){
                                    nuevoMaxNume = tan(cortesMaxImag - std::arg (v->getPuntoNume(numeradorSup, o) + std::arg (v->getPuntoDeno(denominadorInf, o))) - std::arg (plantaNominal)) * o;

                                    if (nuevoMaxNume > numeradorInfNuevo->at(j) && nuevoMaxNume < numeradorSupNuevo->at(j)) {
                                        numeradorSupNuevo->replace(j, nuevoMaxNume);
                                        numeradorUnionY->replace(j, true);
                                    }
                                }
#endif

#if defined(REC_INTER) && defined(REC_MAG)
                                if (!numeradorUnionX->at(j) && datosCortesBoundaries->at(i).recAbajo){
                                    nuevoMinNume = sqrt( pow((cortesMin * abs (v->getPuntoDeno(denominadorInf, o))) /
                                                             (k.y() *  abs (v->getPuntoNume(numeradorSup, o) * plantaNominal)), 2) - pow(o, 2));

                                    if (nuevoMinNume > numeradorInfNuevo->at(j) && nuevoMinNume < numeradorSupNuevo->at(j) && nuevoMinNume < numeradorNuevoInterseccionX->at(j)) {
                                        numeradorNuevoInterseccionX->replace(j, nuevoMinNume);
                                        numeradorInterseccionX->replace(j, true);
                                    }
                                }
#endif
                                numeradorSup->insert(j, n);
#endif
                            }
                        }
                    }
                }


                //Denominador
                if (isVariableDeno){
                    for (qint32 j = 0; j < denominador->size(); j++) {
                        if (denominador->at(j)->isVariable()){

                            if (!datosCortesBoundaries->at(i).uniArriba){

#if (defined(REC_UNION) || defined(REC_INTER)) && (defined(REC_FASE) || defined(REC_MAG))
                                //mag

                                n = denominadorInf->at(j);

                                denominadorInf->remove(j);

#if defined(REC_UNION) && defined(REC_MAG)

                                if (datosCortesBoundaries->at(i).recAbajo){
                                    nuevoMaxDeno = sqrt(pow((k.y() * abs (v->getPuntoNume(numeradorSup, o) * plantaNominal)) /
                                                            (cortesMin * abs(v->getPuntoDeno(denominadorInf, o))), 2) - pow(o, 2));

                                    if (nuevoMaxDeno < denominadorSupNuevo->at(j) && nuevoMaxDeno > denominadorInfNuevo->at(j)) {
                                        denominadorSupNuevo->replace(j, nuevoMaxDeno);
                                        denominadorUnionY->replace(j, true);
                                    }
                                }
#endif

#if defined(REC_INTER) && defined(REC_FASE)
                                if (!denominadorUnionX->at(j) && datosCortesBoundaries->at(i).recIzquierda){
                                    nuevoMinDeno = tan(-cortesMaxImag + std::arg (v->getPuntoNume(numeradorInf, o) - std::arg (v->getPuntoDeno(denominadorSup, o))) + std::arg (plantaNominal)) * o;

                                    if (nuevoMinDeno > denominadorInfNuevo->at(j) && nuevoMinDeno < denominadorSupNuevo->at(j) && nuevoMinDeno < denominadorNuevoInterseccionX->at(j)) {
                                        denominadorNuevoInterseccionX->replace(j, nuevoMinDeno);
                                        denominadorInterseccionX->replace(j, true);
                                    }
                                }
#endif

                                denominadorInf->insert(j, n);

                                //fas
                                n = denominadorSup->at(j);

                                denominadorSup->remove(j);

#if defined(REC_UNION) && defined(REC_FASE)
                                if (datosCortesBoundaries->at(i).recDerecha) {
                                    nuevoMaxDeno = tan(-cortesMinImag + std::arg (v->getPuntoNume(numeradorInf, o) - std::arg (v->getPuntoDeno(denominadorSup, o))) + std::arg (plantaNominal)) * o;

                                    if (nuevoMaxDeno > denominadorInfNuevo->at(j) && nuevoMaxDeno < denominadorSupNuevo->at(j)) {
                                        denominadorSupNuevo->replace(j, nuevoMaxDeno);
                                        denominadorUnionY->replace(j, true);
                                    }
                                }
#endif
#if defined(REC_INTER) && defined(REC_MAG)
                                if (!denominadorUnionX->at(j) && datosCortesBoundaries->at(i).recArriba){
                                    nuevoMinDeno = sqrt(pow((k.x() * abs (v->getPuntoNume(numeradorInf, o) * plantaNominal)) /
                                                            (cortesMax * abs(v->getPuntoDeno(denominadorSup, o))), 2) - pow(o, 2));

                                    if (nuevoMinDeno < denominadorSupNuevo->at(j) && nuevoMinDeno > denominadorInfNuevo->at(j) && nuevoMinDeno < denominadorNuevoInterseccionX->at(j)) {
                                        denominadorNuevoInterseccionX->replace(j, nuevoMinDeno);
                                        denominadorInterseccionX->replace(j, true);
                                    }
                                }
#endif

                                denominadorSup->insert(j, n);

#endif
                            } else {
                                //mag
#if (defined(REC_UNION) || defined(REC_INTER)) && (defined(REC_FASE) || defined(REC_MAG))
                                n = denominadorSup->at(j);

                                denominadorSup->remove(j);
#if defined(REC_UNION) && defined(REC_MAG)
                                if (datosCortesBoundaries->at(i).recArriba){
                                    nuevoMinDeno = sqrt(pow((k.x() * abs (v->getPuntoNume(numeradorInf, o) * plantaNominal)) /
                                                            (cortesMax * abs(v->getPuntoDeno(denominadorSup, o))), 2) - pow(o, 2));
                                    if (nuevoMinDeno < denominadorSupNuevo->at(j) && nuevoMinDeno > denominadorInfNuevo->at(j)) {
                                        denominadorInfNuevo->replace(j, nuevoMinDeno);
                                        denominadorUnionX->replace(j, true);
                                    }
                                }
#endif

#if defined(REC_INTER) && defined(REC_FASE)
                                if (!denominadorUnionY->at(j) && datosCortesBoundaries->at(i).recDerecha){
                                    nuevoMaxDeno = tan(-cortesMinImag + std::arg (v->getPuntoNume(numeradorInf, o) - std::arg (v->getPuntoDeno(denominadorSup, o))) + std::arg (plantaNominal)) * o;

                                    if (nuevoMaxDeno > denominadorInfNuevo->at(j) && nuevoMaxDeno < denominadorSupNuevo->at(j) && nuevoMaxDeno > denominadorNuevoInterseccionY->at(j)) {
                                        denominadorNuevoInterseccionY->replace(j, nuevoMaxDeno);
                                        denominadorInterseccionY->replace(j, true);
                                    }
                                }
#endif
                                denominadorSup->insert(j, n);

                                //imaginario
                                n = denominadorInf->at(j);

                                denominadorInf->remove(j);

#if defined(REC_UNION) && defined(REC_FASE)
                                if (datosCortesBoundaries->at(i).recIzquierda){
                                    nuevoMinDeno = tan(-cortesMaxImag + std::arg (v->getPuntoNume(numeradorInf, o) - std::arg (v->getPuntoDeno(denominadorSup, o))) + std::arg (plantaNominal)) * o;

                                    if (nuevoMinDeno > denominadorInfNuevo->at(j) && nuevoMinDeno < denominadorSupNuevo->at(j)) {
                                        denominadorInfNuevo->replace(j, nuevoMinDeno);
                                        denominadorUnionX->replace(j, true);
                                    }
                                }
#endif

#if defined(REC_INTER) && defined(REC_MAG)
                                if (!denominadorUnionY->at(j) && datosCortesBoundaries->at(i).recAbajo){
                                    nuevoMaxDeno = sqrt(pow((k.y() * abs (v->getPuntoNume(numeradorSup, o) * plantaNominal)) /
                                                            (cortesMin * abs(v->getPuntoDeno(denominadorInf, o))), 2) - pow(o, 2));

                                    if (nuevoMaxDeno > denominadorInfNuevo->at(j) && nuevoMaxDeno < denominadorSupNuevo->at(j) && nuevoMaxDeno > denominadorNuevoInterseccionY->at(j)) {
                                        denominadorNuevoInterseccionY->replace(j, nuevoMaxDeno);
                                        denominadorInterseccionY->replace(j, true);
                                    }
                                }
#endif

                                denominadorInf->insert(j, n);
#endif
                            }
                        }
                    }
                }
            }
        }
    }

    QVector <Var *> * numerador_nuevo;

#if defined(REC_UNION) && defined(REC_INTER)
    if (numeradorInterseccionX || numeradorInterseccionY || numeradorUnionX || numeradorUnionY){
#elif defined (REC_UNION)
    if (numeradorUnionX || numeradorUnionY) {
#elif defined (REC_INTER)
    if (numeradorInterseccionX || numeradorInterseccionY) {
#endif


        numerador_nuevo = new QVector <Var *> ();

        for (qint32 i = 0; i < numerador->size(); i++){

            Var * var_nume_antiguo = numerador->at(i);
            Var * var_nume_nuevo;

            if (var_nume_antiguo->isVariable()){

#ifdef REC_INTER
                if (numeradorInterseccionX->at(i) && numeradorNuevoInterseccionX->at(i) > numeradorInfNuevo->at(i)){
                    numeradorInfNuevo->replace(i, numeradorNuevoInterseccionX->at(i));
                }

                if (numeradorInterseccionY->at(i) && numeradorNuevoInterseccionY->at(i) < numeradorSupNuevo->at(i)){
                    numeradorSupNuevo->replace(i, numeradorNuevoInterseccionY->at(i));
                }
#endif
                var_nume_nuevo = new Var("", QPointF(numeradorInfNuevo->at(i), numeradorSupNuevo->at(i)), 0);
            } else {
                var_nume_nuevo = new Var(var_nume_antiguo->getNominal());
            }

            numerador_nuevo->append(var_nume_nuevo);
        }
    } else {
        numerador_nuevo = numerador;
    }

    QVector <Var *> * denominador_nuevo;

#if defined(REC_UNION) && defined(REC_INTER)
    if (denominadorInterseccionX || denominadorInterseccionY || denominadorUnionX || denominadorUnionY){
#elif defined (REC_UNION)
    if (denominadorUnionX || denominadorUnionY) {
#elif defined (REC_INTER)
    if (denominadorInterseccionX || denominadorInterseccionY){
#endif

        denominador_nuevo = new QVector <Var *> ();
        for (qint32 i = 0; i < denominador->size(); i++){

            Var * var_deno_antiguo = denominador->at(i);
            Var * var_deno_nuevo;

            if (var_deno_antiguo->isVariable()){

#ifdef REC_INTER
                if (denominadorInterseccionX->at(i) && denominadorNuevoInterseccionX->at(i) > denominadorInfNuevo->at(i)){
                    denominadorInfNuevo->replace(i, denominadorNuevoInterseccionX->at(i));
                }

                if (denominadorInterseccionY->at(i) && denominadorNuevoInterseccionY->at(i) < denominadorSupNuevo->at(i)){
                    denominadorSupNuevo->replace(i, denominadorNuevoInterseccionY->at(i));
                }
#endif

                var_deno_nuevo = new Var("", QPointF(denominadorInfNuevo->at(i), denominadorSupNuevo->at(i)), 0);
            } else {
                var_deno_nuevo = new Var(var_deno_antiguo->getNominal());
            }

            denominador_nuevo->append(var_deno_nuevo);
        }

    } else {
        denominador_nuevo = denominador;
    }

#ifdef REC_INTER
    if (kIntersecionX){
        if (kNuevoIntersecionX > kNuevo.x()){
            kNuevo.setX(kNuevoIntersecionX);
        }
    }

    if (kIntersecionY){
        if (kNuevoIntersecionY < kNuevo.y()){
            kNuevo.setY(kNuevoIntersecionY);
        }
    }
#endif

    Sistema * nuevo_sistema = v->invoke(v->getNombre(), numerador_nuevo, denominador_nuevo, new Var("kv", kNuevo, 0), new Var (0.0));
    delete v;


    numeradorSup->clear();
    numeradorInf->clear();
    numeradorInfNuevo->clear();
    numeradorSupNuevo->clear();
#ifdef REC_INTER
    numeradorUnionX->clear();
    numeradorUnionY->clear();
    numeradorInterseccionX->clear();
    numeradorInterseccionY->clear();
    numeradorNuevoInterseccionX->clear();
    numeradorNuevoInterseccionY->clear();
#endif

    denominadorInf->clear();
    denominadorSup->clear();
    denominadorInfNuevo->clear();
    denominadorSupNuevo->clear();
#ifdef REC_INTER
    denominadorUnionX->clear();
    denominadorUnionY->clear();
    denominadorInterseccionX->clear();
    denominadorInterseccionY->clear();
    denominadorNuevoInterseccionX->clear();
    denominadorNuevoInterseccionY->clear();
#endif

    return nuevo_sistema;

}


/*inline Sistema * Algorithm_isaac::aceleratedAntiguo(Sistema * v, QVector <QVector <qreal> *> * cortes_boundarie,
                                                    QVector <flags_box> * tipo_recortes, QVector <bool> * arriba) {

    QVector <Var *> * denominador = v->getDenominador();
    QVector <Var *> * numerador = v->getNumerador();
    QPointF k = v->getK()->getRango();
    QPointF kNuevo = v->getK()->getRango();

    //Creamos los numeradores y denominadores necesarios
    QVector <qreal> * numeradorSup = new QVector <qreal> ();
    QVector <qreal> * numeradorInf = new QVector <qreal> ();
    QVector <qreal> * numeradorSupNuevo = new QVector <qreal> ();
    QVector <qreal> * numeradorInfNuevo = new QVector <qreal> ();

    foreach (Var * var, *numerador) {
        numeradorInf->append(var->getRango().x());
        numeradorSup->append(var->getRango().y());
        numeradorInfNuevo->append(var->getRango().x());
        numeradorSupNuevo->append(var->getRango().y());
    }

    QVector <qreal> * denominadorInf = new QVector <qreal> ();
    QVector <qreal> * denominadorSup = new QVector <qreal> ();
    QVector <qreal> * denominadorInfNuevo = new QVector <qreal> ();
    QVector <qreal> * denominadorSupNuevo = new QVector <qreal> ();

    foreach (Var * var, *denominador) {
        denominadorInf->append(var->getRango().x());
        denominadorSup->append(var->getRango().y());
        denominadorInfNuevo->append(var->getRango().x());
        denominadorSupNuevo->append(var->getRango().y());
    }

    qreal nuevoMinKReal, n, nuevoMinNume, nuevoMaxDeno, o, nuevoMaxKReal, nuevoMaxNume, nuevoMinDeno, cortesMin, cortesMax;
    std::complex <qreal> plantaNominal;


    for (qint32 i = 0; i < omega->size(); i++) {

        if (tipo_recortes->at(i) == ambiguous) {

            o = omega->at(i);
            plantaNominal = plantas_nominales2->at(i);
            cortesMin = cortes_boundarie->at(i)->at(0);
            cortesMax = cortes_boundarie->at(i)->at(1);

            //Análisis de la ganancia

            if (!arriba->at(i)){
                nuevoMinKReal = cortesMin / abs(v->getPunto(numeradorSup, denominadorInf, 1, 0, o) * plantaNominal);

                if (nuevoMinKReal > kNuevo.x() && nuevoMinKReal < kNuevo.y()) {
                    kNuevo.setX(nuevoMinKReal);
                }
            } else {
                nuevoMaxKReal = cortesMax / abs(v->getPunto(numeradorInf, denominadorSup, 1, 0, o) * plantaNominal);

                if (nuevoMaxKReal > kNuevo.x() && nuevoMaxKReal < kNuevo.y()) {
                    kNuevo.setY(nuevoMaxKReal);
                }
            }

            //Numerador
            if (isVariableNume){
                for (qint32 j = 0; j < numerador->size(); j++) {
                    if (numerador->at(j)->isVariable()){
                        if (!arriba->at(i)){
                            n = numeradorSup->at(j);

                            numeradorSup->remove(j);

                            nuevoMinNume = sqrt( pow((cortesMin * abs (v->getPuntoDeno(denominadorInf, o))) /
                                                     (k.y() *  abs (v->getPuntoNume(numeradorSup, o) * plantaNominal)), 2) - pow(o, 2));


                            numeradorSup->insert(j, n);

                            if (nuevoMinNume > numeradorInfNuevo->at(j) && nuevoMinNume < numeradorSupNuevo->at(j)) {
                                numeradorInfNuevo->replace(j, nuevoMinNume);
                            }
                        } else {
                            n = numeradorInf->at(j);

                            numeradorInf->remove(j);

                            nuevoMaxNume = sqrt( pow((cortesMax * abs (v->getPuntoDeno(denominadorSup, o))) /
                                                     (k.x() *  abs (v->getPuntoNume(numeradorInf, o) * plantaNominal)), 2) - pow(o, 2));


                            numeradorInf->insert(j, n);

                            if (nuevoMaxNume > numeradorInfNuevo->at(j) && nuevoMaxNume < numeradorSupNuevo->at(j)) {
                                numeradorSupNuevo->replace(j, nuevoMaxNume);
                            }
                        }
                    }
                }
            }


            //Denominador
            if (isVariableDeno){
                for (qint32 j = 0; j < denominador->size(); j++) {
                    if (denominador->at(j)->isVariable()){

                        if (!arriba->at(i)){
                            n = denominadorInf->at(j);

                            denominadorInf->remove(j);

                            nuevoMaxDeno = sqrt(pow((k.y() * abs (v->getPuntoNume(numeradorSup, o) * plantaNominal)) /
                                                    (cortesMin * abs(v->getPuntoDeno(denominadorInf, o))), 2) - pow(o, 2));

                            denominadorInf->insert(j, n);

                            if (nuevoMaxDeno < denominadorSupNuevo->at(j) && nuevoMaxDeno > denominadorInfNuevo->at(j)) {
                                denominadorSupNuevo->replace(j, nuevoMaxDeno);
                            }
                        } else {
                            n = denominadorSup->at(j);

                            denominadorSup->remove(j);

                            nuevoMinDeno = sqrt(pow((k.x() * abs (v->getPuntoNume(numeradorInf, o) * plantaNominal)) /
                                                    (cortesMax * abs(v->getPuntoDeno(denominadorSup, o))), 2) - pow(o, 2));

                            denominadorSup->insert(j, n);
                            if (nuevoMinDeno < denominadorSupNuevo->at(j) && nuevoMinDeno > denominadorInfNuevo->at(j)) {
                                denominadorInfNuevo->replace(j, nuevoMinDeno);
                            }
                        }
                    }
                }
            }
        }
    }

    QVector <Var *> * numerador_nuevo = new QVector <Var *> ();

    for (qint32 i = 0; i < numerador->size(); i++){

        Var * var_nume_antiguo = numerador->at(i);
        Var * var_nume_nuevo;

        if (var_nume_antiguo->isVariable()){
            var_nume_nuevo = new Var("", QPointF(numeradorInfNuevo->at(i), numeradorSupNuevo->at(i)), 0);
        } else {
            var_nume_nuevo = new Var(var_nume_antiguo->getNominal());
        }

        numerador_nuevo->append(var_nume_nuevo);
    }

    QVector <Var *> * denominador_nuevo = new QVector <Var *> ();

    for (qint32 i = 0; i < denominador->size(); i++){

        Var * var_deno_antiguo = denominador->at(i);
        Var * var_deno_nuevo;

        if (var_deno_antiguo->isVariable()){
            var_deno_nuevo = new Var("", QPointF(denominadorInfNuevo->at(i), denominadorSupNuevo->at(i)), 0);
        } else {
            var_deno_nuevo = new Var(var_deno_antiguo->getNominal());
        }

        denominador_nuevo->append(var_deno_nuevo);
    }

    Sistema * nuevo_sistema = v->invoke(v->getNombre(), numerador_nuevo, denominador_nuevo, new Var("kv", kNuevo, 0), new Var (0.0));
    delete v;


    numeradorSup->clear();
    numeradorInf->clear();
    numeradorInfNuevo->clear();
    numeradorSupNuevo->clear();
    denominadorInf->clear();
    denominadorSup->clear();
    denominadorInfNuevo->clear();
    denominadorSupNuevo->clear();

    return nuevo_sistema;

}*/

//Función que recorta la caja.
/*inline Sistema * Algorithm_isaac::aceleratedAntiguo(Sistema *v, QVector <QVector <qreal> *> * cortes_boundarie,
                                                     QVector <flags_box> * tipo_recortes, QVector <bool> * arriba) {

    qreal kMax = v->getK()->getRango().y();


    qreal nuevokMin = v->getK()->getRango().x();
    qreal nuevokMax = std::numeric_limits<qreal>::lowest();

#ifdef RECSUP
    qreal kMin = v->getK()->getRango().x();
    qreal BMaxRealLineal;
#endif
    qint32 vueltas = omega->size();

#ifdef RECINF
    qreal BminRealLineal;
#endif

    QVector <Var *> * denominador = v->getDenominador();
    QVector <Var *> * numerador = v->getNumerador();

    //Creamos los numeradores y denominadores necesarios
    QVector <qreal> * numeradorSup = new QVector <qreal> ();
    QVector <qreal> * numeradorInf = new QVector <qreal> ();
    QVector <qreal> * nuevoNumeradorInf = new QVector <qreal> ();

#ifdef RECSUP
    QVector <qreal> * nuevoNumeradorSup = new QVector <qreal> ();
#endif

    foreach (Var * var, *numerador) {
        numeradorSup->append(var->getRango().y());
        numeradorInf->append(var->getRango().x());
        nuevoNumeradorInf->append(var->getRango().x());
#ifdef RECSUP
        nuevoNumeradorSup->append(std::numeric_limits<qreal>::lowest());
#endif
    }

    QVector <qreal> * denominadorInf = new QVector <qreal> ();
    QVector <qreal> * denominadorSup = new QVector <qreal> ();
    QVector <qreal> * nuevoDenominadorSup = new QVector <qreal> ();

#ifdef RECSUP
    QVector <qreal> * nuevoDenominadorInf = new QVector <qreal> ();
#endif

    foreach (Var * var, *denominador) {
        denominadorInf->append(var->getRango().x());
        denominadorSup->append(var->getRango().y());
        nuevoDenominadorSup->append(var->getRango().y());
#ifdef RECSUP
        nuevoDenominadorInf->append(std::numeric_limits<qreal>::max());
#endif
    }

    qreal o;
    complex plantaNominal;
#ifdef RECINF
    qreal nuevoMinKReal, nuevoMinNume, nuevoMaxDeno;
#endif

#ifdef RECSUP
    qreal nuevoMinDeno, nuevoMaxKReal, nuevoMaxNume;
#endif
    qreal n;

#ifdef Bi_INTE
    f = new QVector <QVector<qreal> *> ();
#endif

    for (qint32 i = 0; i < vueltas; i++) {

#ifdef Bi_INTE
        QVector<qreal> * vectorFrecuenciasSup = new QVector <qreal>();
#endif

        if (tipo_recortes->at(i) == ambiguous && metaDatosAbierto->at(i)) {

#ifdef RECINF
            BminRealLineal = cortes_boundarie->at(i)->at(0);
#endif

#ifdef RECSUP
            BMaxRealLineal = cortes_boundarie->at(i)->at(1);
#endif

            o = omega->at(i);

            plantaNominal = plantas_nominales->at(i);


            //PARTE REAL
            //Procedemos con el recorte de la K
            //Parte inferior
#ifdef RECINF
            nuevoMinKReal = BminRealLineal / abs(v->getPunto(numeradorSup, denominadorInf, 1, 0, o) * plantaNominal);

            if (nuevoMinKReal > nuevokMin && nuevoMinKReal < nuevokMax) {
                //cout << "k: " << kMin << " nuevo k: " << nuevoMinKReal << endl;
                nuevokMin = nuevoMinKReal;
            }
#endif

#ifdef RECSUP
            //Parte superior
            nuevoMaxKReal = BMaxRealLineal / abs(v->getPunto(numeradorInf, denominadorSup, 1, 0, o) * plantaNominal);

            if (nuevoMaxKReal > nuevokMax && nuevoMaxKReal < kMax) {
                nuevokMax = nuevoMaxKReal;
#ifdef Bi_INTE
                vectorFrecuenciasSup->append(infi);
#endif

            }

#ifdef Bi_INTE
            else if (nuevoMaxKReal > kMin && nuevoMaxKReal < kMax){

                vectorFrecuenciasSup->append(nuevoMaxKReal);


            } else {
                vectorFrecuenciasSup->append(infi);

            }
#endif
#endif

            //Prodecemos con el recorte de la parte real del numerador
            if (isVariableNume) {
                for (qint32 i = 0; i < numeradorSup->size(); i++) {
                    if (numerador->at(i)->isVariable()) {

#ifdef RECINF
                        //Parte inferior
                        n = numeradorSup->at(i);

                        numeradorSup->remove(i);

                        nuevoMinNume = sqrt( pow((BminRealLineal * abs (v->getPuntoDeno(denominadorInf, o))) /
                                                 (kMax *  abs (v->getPuntoNume(numeradorSup, o) * plantaNominal)), 2) - pow(o, 2));


                        numeradorSup->insert(i, n);

                        if (nuevoMinNume > nuevoNumeradorInf->at(i) && nuevoMinNume < n) {
                            //cout << "nume: " << numeradorInf->at(i) << " nuevo nume: " << nuevoMinNume << endl;
                            nuevoNumeradorInf->replace(i, nuevoMinNume);
                        }
#endif
#ifdef RECSUP
                        //Parte superior
                        n = numeradorInf->at(i);

                        numeradorInf->remove(i);

                        nuevoMaxNume = sqrt( pow((BMaxRealLineal * abs (v->getPuntoDeno(denominadorSup, o))) /
                                                 (kMin *  abs (v->getPuntoNume(numeradorInf, o) * plantaNominal)), 2) - pow(o, 2));

                        numeradorInf->insert(i, n);



                        if (nuevoMaxNume > nuevoNumeradorSup->at(i) && nuevoMaxNume < numeradorSup->at(i)) {
                            nuevoNumeradorSup->replace(i, nuevoMaxNume);
#ifdef Bi_INTE
                            vectorFrecuenciasSup->append(infi);
#endif
                        }

#ifdef Bi_INTE
                        else if (nuevoMaxNume > numeradorInf->at(i) && nuevoMaxKReal < numeradorSup->at(i)){

                            vectorFrecuenciasSup->append(nuevoMaxNume);


                        } else {
                            vectorFrecuenciasSup->append(infi);

                        }
#endif
#endif
                    }
#ifdef Bi_INTE
                    else {
                        vectorFrecuenciasSup->append(infi);

                    }
#endif
                }
            }

            //Procedemos con el recorte de la parte real del denominador
            if (isVariableDeno) {
                for (qint32 i = 0; i < denominadorInf->size(); i++) {

                    if (denominador->at(i)->isVariable()) {
#ifdef RECINF
                        //Parte inferior
                        n = denominadorInf->at(i);

                        denominadorInf->remove(i);

                        nuevoMaxDeno = sqrt(pow((kMax * abs (v->getPuntoNume(numeradorSup, o) * plantaNominal)) /
                                                (BminRealLineal * abs(v->getPuntoDeno(denominadorInf, o))), 2) - pow(o, 2));

                        denominadorInf->insert(i, n);
                        if (nuevoMaxDeno < nuevoDenominadorSup->at(i) && nuevoMaxDeno > n) {
                            //cout << "deno: " << denominadorSup->at(i) << " nuevo deno: " << nuevoMaxDeno << endl;
                            nuevoDenominadorSup->replace(i, nuevoMaxDeno);
                        }
#endif
#ifdef RECSUP
                        //Parte superior
                        n = denominadorSup->at(i);

                        denominadorSup->remove(i);

                        nuevoMinDeno = sqrt(pow((kMin * abs (v->getPuntoNume(numeradorInf, o) * plantaNominal)) /
                                                (BMaxRealLineal * abs(v->getPuntoDeno(denominadorSup, o))), 2) - pow(o, 2));

                        denominadorSup->insert(i, n);

                        if (nuevoMinDeno < nuevoDenominadorInf->at(i) && nuevoMinDeno > denominadorInf->at(i)) {
                            nuevoDenominadorInf->replace(i, nuevoMinDeno);
#ifdef Bi_INTE
                            vectorFrecuenciasSup->append(infi);
#endif
                        }
#ifdef Bi_INTE
                        else if (nuevoMinDeno > denominadorInf->at(i) && nuevoMinDeno < denominadorSup->at(i)){

                            vectorFrecuenciasSup->append(nuevoMinDeno);


                        } else {
                            vectorFrecuenciasSup->append(infi);

                        }
#endif

#endif
                    }
#ifdef Bi_INTE
                    else {
                        vectorFrecuenciasSup->append(infi);
                    }
#endif
                }
            }

        } // si la caja para esta frecuencia es ambigua
#ifdef Bi_INTE
        f->append(vectorFrecuenciasSup);
#endif
    } // bucle for




    //Creamos el nuevo controlador
    QVector <Var *> * nuevoNume = new QVector <Var *> ();


    //Numerador
    for (qint32 i = 0; i < numeradorSup->size(); i++) {

        Var * var_nume_nuevo;

        if (numerador->at(i)->isVariable()) {

#ifdef RECSUP
            if (nuevoNumeradorSup->at(i) > numeradorInf->at(i) && nuevoDenominadorSup->at(i) < numeradorSup->at(i)) {
                var_nume_nuevo = new Var("", QPointF(nuevoNumeradorInf->at(i),nuevoNumeradorSup->at(i)), 0);
            } else {
                var_nume_nuevo = new Var("", QPointF(nuevoNumeradorInf->at(i),numeradorSup->at(i)),0);
                //cout << "NumeInf: " + QString::number(numeradorInf->at(i)).toStdString() + " : " + QString::number(nuevoNumeradorInf->at(i)).toStdString() << endl;
            }
#else
#ifdef RECINF
            var_nume_nuevo = new Var("", QPointF(nuevoNumeradorInf->at(i),numeradorSup->at(i)),0);
#endif
#endif

        } else {
            var_nume_nuevo = new Var(numerador->at(i)->getNominal());
        }

        nuevoNume->append(var_nume_nuevo);
    }

    QVector <Var *> * nuevoDeno = new QVector <Var *> ();

    for (qint32 i = 0; i < denominadorInf->size(); i++) {

        Var * var_deno_nuevo;

        if (denominador->at(i)->isVariable()) {
#ifdef RECSUP
            if (nuevoDenominadorInf->at(i) > denominadorInf->at(i) && nuevoDenominadorInf->at(i) < denominadorSup->at(i)) {
                var_deno_nuevo = new Var("", QPointF(nuevoDenominadorInf->at(i),nuevoDenominadorSup->at(i)),0);
            } else {
                var_deno_nuevo = new Var("", QPointF(denominadorInf->at(i),nuevoDenominadorSup->at(i)),0);
            }
#else
#ifdef RECINF
            var_deno_nuevo = new Var("", QPointF(denominadorInf->at(i),nuevoDenominadorSup->at(i)),0);
#endif
#endif

        } else {
            var_deno_nuevo = new Var(denominador->at(i)->getNominal());
        }

        nuevoDeno->append(var_deno_nuevo);
    }

    Sistema * nuevoSistema;

#ifdef RECSUP
    if (nuevokMax > kMin && nuevokMax < kMax) {
        nuevoSistema = v->invoke(v->getNombre(), nuevoNume, nuevoDeno,
                                 new Var("kv", QPointF(nuevokMin, nuevokMax),0), new Var (0));
    } else {
        nuevoSistema = v->invoke(v->getNombre(), nuevoNume, nuevoDeno,
                                 new Var("kv", QPointF(nuevokMin, kMax),0), new Var (0));
    }
#else
#ifdef RECINF
    nuevoSistema = v->invoke(v->getNombre(), nuevoNume, nuevoDeno,
                             new Var("kv", QPointF(nuevokMin, kMax),0), new Var (0));
#endif
#endif


    delete v;

    numeradorSup->clear();
    numeradorInf->clear();
    denominadorInf->clear();
    denominadorSup->clear();
    nuevoDenominadorSup->clear();
    nuevoNumeradorInf->clear();

#ifdef RECSUP
    nuevoNumeradorSup->clear();
    nuevoDenominadorInf->clear();
#endif

#ifdef Bi_INTE
    biseccion(v);
    f->clear();
#endif

    return nuevoSistema;
}*/


inline void Algorithm_isaac::comprobarVariables(Sistema *controlador) {
    bool b = true;

    foreach(Var * var, *controlador->getNumerador()) {
        if (var->isVariable()) {
            b = false;
        }
    }

    isVariableNume = !b;

    b = true;

    foreach(Var * var, *controlador->getDenominador()) {
        if (var->isVariable()) {
            b = false;
        }
    }

    isVariableDeno = !b;
}


//Función que divide la caja en dos clásica.

inline FC::return_bisection Algorithm_isaac::split_box_bisection(Sistema *current_controlador) {

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
    qint32 cont = 0;

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



//Funcion que divida la caja en dos avanzada.

inline FC::return_bisection Algorithm_isaac::split_box_bisection_avanced(Sistema *current_controlador) {

    QVector <Var *> * numerador = current_controlador->getNumerador();
    QVector <Var *> * denominador = current_controlador->getDenominador();

    QPointF k = current_controlador->getK()->getRango();
    Var * ret = current_controlador->getRet();

    QVector <Var *> * numeradorCopia = new QVector <Var *> ();
    QVector <Var *> * denominadorCopia = new QVector <Var *> ();

    QString nombre = current_controlador->getNombre();

    qreal menor_punto_medio = 0, menor_area = 0;
    qint32 seleccionado = 0;


    Sistema * v1, * v2;
    struct FC::return_bisection retur;

    qreal punto_medio_k = k.x() + (k.y() - k.x()) / 2;

    cinterval nume_box = conversion->get_box_nume(numerador, omega->at(0), current_controlador->getClass(), false);
    cinterval deno_box = conversion->get_box_deno(denominador, omega->at(0), current_controlador->getClass(), false);

    //Comprobamos la k

    interval k1(k.x(), punto_medio_k);
    //interval k2 (punto_medio_k, k.y());

    cinterval a = k1 * (nume_box / deno_box);
    qreal s = _double(diam(Re(a)) * diam(Im(a)));
    //qreal s1 = area(k2 * (nume_box / deno_box));
    //cout << "area k: " << s << endl;

    menor_area = s;
    seleccionado = -1;
    menor_punto_medio = punto_medio_k;

    qint32 contador = 0;

    complex complejo(0, omega->at(0));

    //Comprobamos el numerador

    if (numerador->size() > 1) {

        for (qint32 i = 0; i < numerador->size(); i++) {
            numeradorCopia->append(numerador->at(i)->clone());

            if (numerador->at(i)->isVariable()) {

                Var * v = numerador->at(i);
                numerador->remove(i);

                qreal punto_medio = v->getRango().x() + (v->getRango().y() - v->getRango().x()) / 2;

                //cinterval nume_box_nuevo = conversion->get_box_nume(numerador, omega->at(0), current_controlador->getClass(), true);

                //cinterval nume_uno = nume_box_nuevo * (complejo + interval(v->getRango().x(), punto_medio));
                //cinterval nume_dos = nume_box_nuevo * (complejo + interval(punto_medio, v->getRango().y()));

                a = k1 * (nume_box / deno_box);
                s = _double(diam(Re(a)) * diam(Im(a)));

                //s1 = area(k_box * (nume_dos / deno_box));

                numerador->insert(i, v);

                if (s < menor_area) {
                    menor_area = s;
                    seleccionado = contador;
                    menor_punto_medio = punto_medio;
                }
            }
            contador++;
        }
    } else if (numerador->size() > 0) {
        numeradorCopia->append(numerador->at(0)->clone());

        if (numerador->at(0)->isVariable()) {

            QPointF v = numerador->at(0)->getRango();
            qreal punto_medio = v.x() + (v.y() - v.x()) / 2;

            //cinterval nume_uno = (complejo + interval(v.x(), punto_medio));
            //cinterval nume_dos = (complejo + interval(punto_medio, v.y()));

            a = k1 * (nume_box / deno_box);
            s = _double(diam(Re(a)) * diam(Im(a)));
            //cout << "area nume: " << s << endl;
            //s1 = area(k_box * (nume_dos / deno_box));

            if (s < menor_area) {
                menor_area = s;
                seleccionado = contador;
                menor_punto_medio = punto_medio;
            }
        }
        contador++;
    }

    //Comprobamos el denominador

    if (denominador->size() > 1) {

        for (qint32 i = 0; i < denominador->size(); i++) {
            denominadorCopia->append(denominador->at(i)->clone());

            if (denominador->at(i)->isVariable()) {

                Var * v = denominador->at(i);
                denominador->remove(i);

                qreal punto_medio = v->getRango().x() + (v->getRango().y() - v->getRango().x()) / 2;

                //cinterval deno_box_nuevo = conversion->get_box_nume(denominador, omega->at(0), current_controlador->getClass(), true);

                //cinterval deno_uno = deno_box_nuevo * (complejo + interval(v->getRango().x(), punto_medio));
                //cinterval deno_dos = deno_box_nuevo * (complejo + interval(punto_medio, v->getRango().y()));

                a = k1 * (nume_box / deno_box);
                s = _double(diam(Re(a)) * diam(Im(a)));
                //s1 = area(k_box * (nume_box / deno_dos));

                denominador->insert(i, v);

                if (s < menor_area) {
                    menor_area = s;
                    seleccionado = contador;
                    menor_punto_medio = punto_medio;
                }
            }
            contador++;
        }
    } else if (denominador->size() > 0) {
        denominadorCopia->append(denominador->at(0)->clone());

        if (denominador->at(0)->isVariable()) {

            QPointF v = denominador->at(0)->getRango();
            qreal punto_medio = v.x() + (v.y() - v.x()) / 2;

            //cinterval deno_uno = (complejo + interval(v.x(), punto_medio));
            //cinterval deno_dos = (complejo + interval(punto_medio, v.y()));

            a = k1 * (nume_box / deno_box);
            s = _double(diam(Re(a)) * diam(Im(a)));
            //s1 = area(k_box * (nume_box / deno_dos));
            if (s < menor_area) {
                menor_area = s;
                seleccionado = contador;
                menor_punto_medio = punto_medio;
            }
        }
    }


    if (seleccionado == -1) {

        //cout << "partimos K" << endl;

        Var * k1 = new Var("kv", QPointF(k.x(), menor_punto_medio), k.x());
        Var * k2 = new Var("kv", QPointF(menor_punto_medio, k.y()), menor_punto_medio);

        delete current_controlador->getK();

        v1 = current_controlador->invoke(nombre, numerador, denominador, k1, ret);
        v2 = current_controlador->invoke(nombre, numeradorCopia, denominadorCopia, k2, ret->clone());
    } else if (seleccionado < numerador->size()) {

        //cout << "partimos numerador" << endl;

        Var * variable = numerador->at(seleccionado);

        numeradorCopia->replace(seleccionado, new Var("", QPointF(variable->getRango().x(), menor_punto_medio), variable->getRango().x()));

        numerador->replace(seleccionado, new Var("", QPointF(menor_punto_medio, variable->getRango().y()), menor_punto_medio));


        v1 = current_controlador->invoke(nombre, numerador, denominador, current_controlador->getK(), ret);
        v2 = current_controlador->invoke(nombre, numeradorCopia, denominadorCopia, current_controlador->getK()->clone(), ret->clone());

        delete variable;

    } else {

        //cout << "partimos denominador" << endl;

        seleccionado -= numerador->size();

        Var * variable = denominador->at(seleccionado);

        denominadorCopia->replace(seleccionado, new Var("", QPointF(variable->getRango().x(), menor_punto_medio), variable->getRango().x()));

        denominador->replace(seleccionado, new Var("", QPointF(menor_punto_medio, variable->getRango().y()), menor_punto_medio));

        v1 = current_controlador->invoke(nombre, numerador, denominador, current_controlador->getK(), ret);
        v2 = current_controlador->invoke(nombre, numeradorCopia, denominadorCopia, current_controlador->getK()->clone(), ret->clone());

        delete variable;

    }

    retur.v1 = v1;
    retur.v2 = v2;
    retur.descartado = false;

    current_controlador->borrar();

    delete current_controlador;

    return retur;
}
