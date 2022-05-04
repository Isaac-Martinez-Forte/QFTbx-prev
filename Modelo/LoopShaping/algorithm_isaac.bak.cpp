#include "algorithm_isaac.h"

#include<iostream>
#include<stdlib.h>
#include<time.h>

#define RECINF
#define RECSUP
#define Bi_INTE
//#define BE
//#define Imprimir
//#define Exportar

#define infi  10000000000

using namespace tools;
using namespace intervalares;
using namespace std;

Algorithm_isaac::Algorithm_isaac()
{

}

Algorithm_isaac::~Algorithm_isaac()
{

}


void Algorithm_isaac::set_datos(Sistema *planta, Sistema *controlador, QVector<qreal> * omega, DatosBound *boundaries,
                                qreal epsilon, QVector<QVector<QVector<QPointF> *> *> *reunBounHash,
                                bool depuracion __attribute__((unused)), bool hilos, QVector<qreal>*radiosBoundariesMayor,
                                QVector<qreal> *radiosBoundariesMenor, QVector<QPointF> *centros, bool biseccion_avanzada, bool deteccion_avanzada) {
    this->planta = planta;
    this->controlador = controlador->clone();
    this->omega = omega;
    this->boundaries = boundaries;
    this->epsilon = epsilon;
    this->reunBounHash = reunBounHash;

    this->metaDatosArriba = boundaries->getMetaDatosArriba();
    this->metaDatosAbierto = boundaries->getMetaDatosAbierta();

    this->tamFas = boundaries->getTamFas()-1;
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
    } else {
        deteccionViolacion = &DeteccionViolacionBoundaries::deteccionViolacionCajaNiNi;
        Nyquist = false;
    }
}


//Función principal del algoritmo
bool Algorithm_isaac::init_algorithm() {

    lista = new ListaOrdenada ();

    conversion = new Natura_Interval_extension ();

    Tripleta * tripleta;
    Tripleta * actual;
    Tripleta * actual2;

    struct FC::return_bisection retur;
    deteccion = new DeteccionViolacionBoundaries();

    plantas_nominales = new QVector <complex <qreal> > ();

    foreach (qreal o, *omega) {
        plantas_nominales->append(planta->getPunto(o));
    }

    comprobarVariables(controlador);



    //lista->insertar(new Tripleta(controlador->getK()->getRango().x(), controlador));

    actual = check_box_feasibility(controlador);

    if (actual == NULL) {
        menerror("El espacio de parámetros inicial del controlador no es válido.", "Loop Shaping");

        delete conversion;
        delete lista;
        delete deteccion;

        return false;
    }


    if (flag_return == feasible || FC::if_less_epsilon(actual->getSistema(), epsilon, omega, conversion)) {
        if (flag_return == ambiguous) {
            controlador_retorno = FC::guardarControlador(actual->getSistema(), false);
        } else {
            controlador_retorno = FC::guardarControlador(actual->getSistema(), true);
        }

        delete conversion;
        delete lista;
        delete deteccion;

        return true;
    }

    lista->insertar(actual);


    if (!hilos) {

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

            /*cout << endl << "uno" << endl;

            cout << "k: [" + QString::number(tripleta->sistema->getK()->getRango().x()).toStdString() + ", " +
                    QString::number(tripleta->sistema->getK()->getRango().y()).toStdString() + "] ----- ";

            cout << "nume: [" + QString::number(tripleta->sistema->getNumerador()->at(0)->getRango().x()).toStdString() + ", " +
                    QString::number(tripleta->sistema->getNumerador()->at(0)->getRango().y()).toStdString() + "] ----- " ;*/


            /*cout << "nume: [" + QString::number(tripleta->sistema->getNumerador()->at(1)->getRango().x()).toStdString() + ", " +
                    QString::number(tripleta->sistema->getNumerador()->at(1)->getRango().y()).toStdString() + "] ----- " ;

            cout << "nume: [" + QString::number(tripleta->sistema->getNumerador()->at(2)->getRango().x()).toStdString() + ", " +
                    QString::number(tripleta->sistema->getNumerador()->at(2)->getRango().y()).toStdString() + "] ----- " ;

            cout << "deno: [" + QString::number(tripleta->sistema->getDenominador()->at(0)->getRango().x()).toStdString() + ", " +
                    QString::number(tripleta->sistema->getDenominador()->at(0)->getRango().y()).toStdString() + "] ----- ";

            cout << "deno: [" + QString::number(tripleta->sistema->getDenominador()->at(1)->getRango().x()).toStdString() + ", " +
                    QString::number(tripleta->sistema->getDenominador()->at(1)->getRango().y()).toStdString() + "] ----- ";

            cout << "deno: [" + QString::number(tripleta->sistema->getDenominador()->at(2)->getRango().x()).toStdString() + ", " +
                    QString::number(tripleta->sistema->getDenominador()->at(2)->getRango().y()).toStdString() + "]" << endl;*/

            //Split blox
            retur = (this->*split_box)(tripleta->sistema);
            delete tripleta;


            actual = check_box_feasibility(retur.v1);

            if (actual != NULL) {

                if (flag_return == feasible || FC::if_less_epsilon(actual->getSistema(), epsilon, omega, conversion)) {
                    if (flag_return == ambiguous) {
                        controlador_retorno = FC::guardarControlador(actual->getSistema(), false);
                    } else {
                        controlador_retorno = FC::guardarControlador(actual->getSistema(), true);
                    }

                    delete conversion;
                    delete lista;
                    delete deteccion;

                    return true;
                }

                lista->insertar(actual);

            }


            actual2 = check_box_feasibility(retur.v2);

            if (actual2 != NULL) {

                if (flag_return == feasible || FC::if_less_epsilon(actual2->getSistema(), epsilon, omega, conversion)) {
                    if (flag_return == ambiguous) {
                        controlador_retorno = FC::guardarControlador(actual2->getSistema(), false);
                    } else {
                        controlador_retorno = FC::guardarControlador(actual2->getSistema(), true);
                    }

                    delete conversion;
                    delete lista;
                    delete deteccion;

                    return true;
                }

                lista->insertar(actual2);

            }


            /*cout <<endl << "dos" << endl;

            cout << "k: [" + QString::number(actual->getK()->getRango().x()).toStdString() + ", " +
                    QString::number(actual->getK()->getRango().y()).toStdString() + "] ----- ";

            cout << "nume: [" + QString::number(actual->getNumerador()->at(0)->getRango().x()).toStdString() + ", " +
                    QString::number(actual->getNumerador()->at(0)->getRango().y()).toStdString() + "] ----- " ;*/


        }
    } else {
        //Sección hilos
        terminacionCorrecta = false;
        std::cout << "Ejecución por hilos" << std::endl;

        vectorHilos = new QVector <Algorithm_isaac_hilo *> ();

        qint32 nHilos = QThread::idealThreadCount();

        for (qint32 i = 0; i < nHilos; i++) {
            Algorithm_isaac_hilo * h = new Algorithm_isaac_hilo(this);
            vectorHilos->append(h);
            h->start();
        }

        mutexTerminar.lock();
        mutexTerminar.lock();

        for (qint32 i = 0; i < nHilos; i++) {
            Algorithm_isaac_hilo * h = vectorHilos->at(i);
            h->terminate();
            h->wait();
            delete h;
        }

        mutexTerminar.unlock();

        mutexEnding.unlock();
        delete semaforo;
    }

    return true;
}

//Función que retorna el controlador.
Sistema * Algorithm_isaac::getControlador() {
    return controlador_retorno;
}


//Función que comprueba si la caja actual es feasible, infeasible o ambiguous.
inline Tripleta * Algorithm_isaac::check_box_feasibility(Sistema * controlador) {

    using namespace std;

    struct datos_caja datos;

    flags_box flag_final = feasible;

    //////////////////////////////////////////////////////////////////////////////////////////////////
    /*qreal maglineal = 0;

     QVector< QVector<QPointF> * > * boun = boundaries->getBoundariesReun();

     QVector< QVector<QPointF> * > * nuevosBoundariesReun =
             new QVector< QVector<QPointF> * > ();


     QVector< QVector< QVector<QPointF> * > * > * nuevoHash_inter = new QVector< QVector< QVector<QPointF> * > * > ();

     foreach (auto vector, *boun) {

         QVector<QPointF> * nuevoVector = new QVector<QPointF>  ();

         QVector <QVector <QPointF> * > * nuevoHash = new QVector <QVector <QPointF> *> ();

         foreach (auto p, *vector) {
             maglineal = pow(10.,p.y()/20.);

             QPointF punto (maglineal * cos (p.x() * M_PI / 180.),
                            maglineal * sin (p.x() * M_PI / 180.));

             nuevoVector->append(punto);


         }

         nuevoHash_inter->append(nuevoHash);

         nuevosBoundariesReun->append(nuevoVector);
     }




     QPointF nuevoDatosFas ((boundaries->getDatosFas().x() * M_PI) / 180, 0);

     QPointF datosMag = boundaries->getDatosMag();

     QPointF nuevosDatosMag (pow(10,datosMag.x()/20), pow(10,datosMag.y()/20));

     DatosBound * nuevoBoundaries = new DatosBound (boundaries->getBoundaries(), boundaries->getMetaDatosAbierta(),
                                                    boundaries->getMetaDatosArriba(), boundaries->getTamFas(),
                                                    nuevoDatosFas, nuevosBoundariesReun,
                                                    nuevoHash_inter,
                                                    boundaries->getTamMag(), nuevosDatosMag);

    QVector <QVector <QPointF> *> * vectorCajas = new QVector <QVector <QPointF> *> ();
    QVector <qreal> * vectorareas = new QVector <qreal> ();*/


    //////////////////////////////////////////////////////////////////////////////////////////////////


#ifdef DEBUG
    QVector <QVector <QPointF> * > * puntos;

    if (depuracion)
        puntos = new QVector <QVector <QPointF> * > ();
#endif

    QVector <QVector <qreal> * > * cortes_boundaries_vector = new QVector <QVector <qreal> *> ();
    QVector <flags_box> * tipo_recortes_vector = new QVector <flags_box> ();

    Tripleta * t = new Tripleta();
    qint32 nFeasible = 0;
    qreal porFeasible = 0;


    for (qint32 k = 0; k < omega->size(); k++) {

        datos = (deteccion->*deteccionViolacion)(conversion->get_box(controlador,omega->at(k), plantas_nominales->at(k), Nyquist) , reunBounHash->at(k),
                                                 tamFas, metaDatosAbierto->at(k), metaDatosArriba->at(k), boundaries->getDatosFas().y() - boundaries->getDatosFas().x());

        tipo_recortes_vector->append(datos.flag);
        porFeasible += datos.porcentajeFeasible;


#ifdef DEBUG
        if (depuracion)
            puntos->append(deteccion_violacion->getPuntos());
#endif

        //////////////////////////////////////////////////////////////////////////
        /*QVector <QPointF> * v = new QVector <QPointF> ();

        cinterval <qreal> caja = conversion->get_box(controlador,omega->at(k), plantas_nominales->at(k), false);

        v->append(QPointF(caja.im.inf, caja.re.inf));
        v->append(QPointF(caja.im.inf, caja.re.sup));
        v->append(QPointF(caja.im.sup, caja.re.sup));
        v->append(QPointF(caja.im.sup, caja.re.inf));

        //FC::mostrar_diagramaBox(v, this->omega, boundaries);
        vectorCajas->append(v);
        //vectorareas->append(area(caja));*/
        /////////////////////////////////////////////////////////////////////////////

        if (datos.flag == infeasible) {
            delete controlador;
            cortes_boundaries_vector->clear();
            tipo_recortes_vector->clear();
            flag_return = infeasible;
            return NULL;
        }

        if (datos.flag == ambiguous) {
            flag_final = ambiguous;
        } else {
            nFeasible++;
        }

        cortes_boundaries_vector->append(datos.minimosMaximos);

    }

    t->setFlag(tipo_recortes_vector);
    t->setNumeroFeasibles(nFeasible);


#ifdef BE
    t->setIndex(-(20 * nFeasible) + 10 * t->getPorcentajeFeasible());
#else
    t->setIndex(controlador->getK()->getRango().x());
#endif

#ifdef DEBUG
    if (depuracion)
        mostrar_diagrama(puntos, omega, boundaries);
#endif

    ///////////////////////////////////////////////////////////////////

    /*ViewBoundReun * view = FC::mostrar_diagrama(vectorCajas, this->omega, boundaries);
    view->exec();
    vectorCajas->clear();*/

    /////////////////////////////////////////////////////////////////////

    if (flag_final == ambiguous) {
        controlador = acelerated(controlador, cortes_boundaries_vector, tipo_recortes_vector);
    }

    t->setSistema(controlador);

    //////////////////////////////////////////////////////////////////////////
    /*vectorCajas = new QVector <QVector <QPointF> *> ();

    for (qint32 k = 0; k < omega->size(); k++) {
        QVector <QPointF> * v = new QVector <QPointF> ();

        cinterval <qreal> caja = conversion->get_box(controlador,omega->at(k), plantas_nominales->at(k), false);

        v->append(QPointF(caja.im.inf, caja.re.inf));
        v->append(QPointF(caja.im.inf, caja.re.sup));
        v->append(QPointF(caja.im.sup, caja.re.sup));
        v->append(QPointF(caja.im.sup, caja.re.inf));

        //FC::mostrar_diagramaBox(v, this->omega, boundaries);
        vectorCajas->append(v);
        //cout << vectorareas->at(k) << ";" << area(caja) << ";" << k << ";" << endl;
    }

    //FC::mostrar_diagrama2(vectorCajas, this->omega, boundaries, view);
    vectorCajas->clear();
    vectorareas->clear();*/

    /////////////////////////////////////////////////////////////////////////////


    cortes_boundaries_vector->clear();

    flag_return = flag_final;

    return t;
}


//Función que recorta la caja.
inline Sistema * Algorithm_isaac::acelerated(Sistema *v, QVector <QVector <qreal> *> * cortes_boundarie,
                                             QVector <flags_box> * tipo_recortes) {



    qreal kMax = v->getK()->getRango().y();


    qreal nuevokMin = v->getK()->getRango().x();
    qreal nuevokMax = numeric_limits<qreal>::min();

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
        nuevoNumeradorSup->append(numeric_limits<qreal>::min());
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
        nuevoDenominadorInf->append(numeric_limits<qreal>::max());
#endif
    }

    qreal o;
    complex <qreal> plantaNominal;
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

    v = nuevoSistema;

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

    return v;
}


inline Sistema * Algorithm_isaac::biseccion(Sistema *s) {

    QVector <qreal> * vector = new QVector <qreal> ();
    QVector <qint32> * n = new QVector <qint32> ();

    int n1 = 0;

    for (int i = 0; i < f->size(); i++){
        QVector <qreal> * v = f->at(i);
        qreal min = infi;

        for (int j = 0; j < v->size(); j++){
            if (v->at(j) < min){
                n1 = j;
                min = v->at(j);
            }
        }
        vector->append(min);
        n->append(n1);
    }

    QVector <Var *> * numerador = s->getNumerador();
    QVector <Var *> * denominador = s->getDenominador();

    Var * k = s->getK();
    Var * ret = s->getRet();

    QString nombre = s->getNombre();

    Sistema * v1;

    for (int i = 0; i < vector->size(); i++) {

        int mayor_pos = n->at(i) -1;
        qreal mayor_rango = vector->at(i);

        if (mayor_rango != infi){

            cout << "Entra para omega: " << omega->at(i) << " variable: " << mayor_rango << endl;

            if (mayor_pos == -1) {
                Var * k1 = new Var ("kv", QPointF(mayor_rango, k->getRango().y()), mayor_rango, k->getGranularidad());

                v1 = s->invoke(nombre, numerador, denominador, k1, ret);
            } else if (mayor_pos < numerador->size()) {

                Var * variable = numerador->at(mayor_pos);
                numerador->replace(mayor_pos, new Var("", QPointF(mayor_rango, variable->getRango().y()), mayor_rango, variable->getGranularidad()));

                v1 = s->invoke(nombre, numerador, denominador, k, ret);

            } else {
                mayor_pos -= numerador->size();

                Var * variable = denominador->at(mayor_pos);

                denominador->replace(mayor_pos, new Var("", QPointF(mayor_rango, variable->getRango().y()),mayor_rango, variable->getGranularidad()));

                v1 = s->invoke(nombre, numerador, denominador, k, ret);
            }

            QVector <QVector <QPointF> *> *vectorCajas = new QVector <QVector <QPointF> *> ();

            for (qint32 k = 0; k < omega->size(); k++) {
                QVector <QPointF> * v = new QVector <QPointF> ();

                cinterval <qreal> caja = conversion->get_box(v1,omega->at(k), plantas_nominales->at(k), false);

                v->append(QPointF(caja.im.inf, caja.re.inf));
                v->append(QPointF(caja.im.inf, caja.re.sup));
                v->append(QPointF(caja.im.sup, caja.re.sup));
                v->append(QPointF(caja.im.sup, caja.re.inf));

                vectorCajas->append(v);
            }

            ViewBoundReun * v = FC::mostrar_diagrama(vectorCajas, this->omega, boundaries);
            v->exec();
            vectorCajas->clear();

            s = v1;
        } else {
            cout << "NO entra para omega: " << omega->at(i) << endl;
        }
    }


    cout << "-------------------------------------" << endl;

    return s;

}


inline void Algorithm_isaac::comprobarVariables(Sistema *controlador) {
    bool b = true;

    foreach (Var * var, *controlador->getNumerador()) {
        if (var->isVariable()) {
            b = false;
        }
    }

    isVariableNume = !b;

    b = true;

    foreach (Var * var, *controlador->getDenominador()) {
        if (var->isVariable()) {
            b = false;
        }
    }

    isVariableDeno = !b;
}


//Función que divide la caja en dos clásica.
inline FC::return_bisection Algorithm_isaac::split_box_bisection(Sistema * current_controlador) {

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

            /*if (lon < (epsilon / 10)) {
                retur.descartado = true;
                delete current_controlador;
                return retur;
            }*/

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

            /*if (lon < (epsilon / 10)) {
                retur.descartado = true;
                delete current_controlador;
                return retur;
            }*/

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

        /*if (lon < (epsilon / 10)) {
            retur.descartado = true;
            delete current_controlador;
            return retur;
        }*/

        if (lon > mayor_rango) {
            mayor_pos = -1;
            mayor_rango = lon;
        }
    }


    if (mayor_pos == -1) {
        qreal dis = k->getRango().x();
        Var * k1 = new Var ("kv", QPointF(dis, dis + (mayor_rango / 2)), dis, k->getGranularidad());
        dis += mayor_rango / 2;
        Var * k2 = new Var ("kv", QPointF(dis, k->getRango().y()), dis, k->getGranularidad());

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
inline FC::return_bisection Algorithm_isaac::split_box_bisection_avanced(Sistema * current_controlador) {

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

    cinterval <qreal> nume_box = conversion->get_box_nume (numerador, omega->at(0), current_controlador->getClass(), true);
    cinterval <qreal> deno_box = conversion->get_box_deno (denominador, omega->at(0), current_controlador->getClass(), true);

    interval <qreal> k_box (k.x(), k.y());

    //Comprobamos la k

    interval <qreal> k1 (k.x(), punto_medio_k);
    //interval <qreal> k2 (punto_medio_k, k.y());

    qreal s = area (k1 * (nume_box / deno_box));
    //qreal s1 = area(k2 * (nume_box / deno_box));
    //cout << "area k: " << s << endl;

    menor_area = s;
    seleccionado = -1;
    menor_punto_medio = punto_medio_k;

    qint32 contador = 0;

    complex<qreal> complejo (0, omega->at(0));

    //Comprobamos el numerador

    if (numerador->size() > 1) {

        for (qint32 i = 0; i < numerador->size(); i++) {
            numeradorCopia->append(numerador->at(i)->clone());

            if (numerador->at(i)->isVariable()) {

                Var * v = numerador->at(i);
                numerador->remove(i);

                qreal punto_medio = v->getRango().x() + (v->getRango().y() - v->getRango().x()) / 2;

                cinterval <qreal> nume_box_nuevo = conversion->get_box_nume (numerador, omega->at(0), current_controlador->getClass(), true);

                cinterval <qreal> nume_uno = nume_box_nuevo * (complejo + interval <qreal>(v->getRango().x(), punto_medio));
                //cinterval <qreal> nume_dos = nume_box_nuevo * (complejo + interval <qreal>(punto_medio, v->getRango().y()));

                s = area(k_box * (nume_uno / deno_box));

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

            cinterval <qreal> nume_uno = (complejo + interval <qreal>(v.x(), punto_medio));
            //cinterval <qreal> nume_dos = (complejo + interval <qreal>(punto_medio, v.y()));

            s = area(k_box * (nume_uno / deno_box));
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

                cinterval <qreal> deno_box_nuevo = conversion->get_box_nume (denominador, omega->at(0), current_controlador->getClass(), true);

                cinterval <qreal> deno_uno = deno_box_nuevo * (complejo + interval <qreal>(v->getRango().x(), punto_medio));
                //cinterval <qreal> deno_dos = deno_box_nuevo * (complejo + interval <qreal>(punto_medio, v->getRango().y()));

                s = area(k_box * (nume_box / deno_uno));
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

            cinterval <qreal> deno_uno = (complejo + interval <qreal>(v.x(), punto_medio));
            //cinterval <qreal> deno_dos = (complejo + interval <qreal>(punto_medio, v.y()));

            s = area(k_box * (nume_box / deno_uno));
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

        Var * k1 = new Var ("kv", QPointF(k.x(), menor_punto_medio), k.x());
        Var * k2 = new Var ("kv", QPointF(menor_punto_medio, k.y()), menor_punto_medio);

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


/*cout << "k: [" + QString::number(tripleta->sistema->getK()->getRango().x()).toStdString() + ", " +
        QString::number(tripleta->sistema->getK()->getRango().y()).toStdString() + "] " << endl;

cout << "nume: [" + QString::number(tripleta->sistema->getNumerador()->at(0)->getRango().x()).toStdString() + ", " +
        QString::number(tripleta->sistema->getNumerador()->at(0)->getRango().y()).toStdString() + "]" << endl;

cout << "deno: [" + QString::number(tripleta->sistema->getDenominador()->at(0)->getRango().x()).toStdString() + ", " +
        QString::number(tripleta->sistema->getDenominador()->at(0)->getRango().y()).toStdString() + "]" << endl;*/


/* if (controlador_actual == NULL){
    cout << "infeasible" << endl;
}*/


/* qreal maglineal = 0;

QVector< QVector<QPointF> * > * boun = boundaries->getBoundariesReun();

QVector< QVector<QPointF> * > * nuevosBoundariesReun =
        new QVector< QVector<QPointF> * > ();


QVector< QVector< QVector<QPointF> * > * > * nuevoHash_inter = new QVector< QVector< QVector<QPointF> * > * > ();

foreach (auto vector, *boun) {

    QVector<QPointF> * nuevoVector = new QVector<QPointF>  ();

    QVector <QVector <QPointF> * > * nuevoHash = new QVector <QVector <QPointF> *> ();

    foreach (auto p, *vector) {
        maglineal = pow(10.,p.y()/20.);

        QPointF punto (maglineal * cos (p.x() * M_PI / 180.),
                       maglineal * sin (p.x() * M_PI / 180.));

        nuevoVector->append(punto);


    }

    nuevoHash_inter->append(nuevoHash);

    nuevosBoundariesReun->append(nuevoVector);
}




QPointF nuevoDatosFas ((boundaries->getDatosFas().x() * M_PI) / 180, 0);

QPointF datosMag = boundaries->getDatosMag();

QPointF nuevosDatosMag (pow(10,datosMag.x()/20), pow(10,datosMag.y()/20));

DatosBound * nuevoBoundaries = new DatosBound (boundaries->getBoundaries(), boundaries->getMetaDatosAbierta(),
                                               boundaries->getMetaDatosArriba(), boundaries->getTamFas(),
                                               nuevoDatosFas, nuevosBoundariesReun,
                                               nuevoHash_inter,
                                               boundaries->getTamMag(), nuevosDatosMag);

QVector <QVector <QPointF> *> * vectorCajas = new QVector <QVector <QPointF> *> ();*/


/* QVector <QPointF> * v = new QVector <QPointF> ();

cinterval <qreal> caja = conversion->get_box(controlador,o, planta->getPunto(o), true);

v->append(QPointF(caja.re.inf, caja.im.inf));
v->append(QPointF(caja.re.inf, caja.im.sup));
v->append(QPointF(caja.re.sup, caja.im.inf));
v->append(QPointF(caja.re.sup, caja.im.sup));

//mostrar_diagramaBox(v, this->omega, nuevoBoundaries);

vectorCajas->append(v);*/

