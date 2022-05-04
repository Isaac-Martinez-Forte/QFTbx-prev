#include "algorithm_nandkishor.h"

using namespace tools;
using namespace cxsc;
using namespace FC;

Algorithm_nandkishor::Algorithm_nandkishor()
{

}

Algorithm_nandkishor::~Algorithm_nandkishor()
{

}


void Algorithm_nandkishor::set_datos(Sistema *planta, Sistema *controlador, QVector<qreal> * omega, DatosBound *boundaries,
                                     qreal epsilon, QVector<QVector<QVector<QPointF> *> *> *reunBounHash,
                                     qreal delta, qint32 inicializacion){

    this->planta = planta;
    this->controlador = controlador->clone();
    this->controlador_inicial = controlador->clone();
    this->omega = omega;
    this->boundaries = boundaries;
    this->epsilon = epsilon;
    this->reunBounHash = reunBounHash;

    this->metaDatosArriba = boundaries->getMetaDatosArriba();
    this->metaDatosAbierto = boundaries->getMetaDatosAbierta();

    this->tamFas = boundaries->getTamFas() -1;
    this->depuracion = true;

    this->delta = delta;

    this->ini = (tipoInicializacion) inicializacion;


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


//Función principal del algoritmo
bool Algorithm_nandkishor::init_algorithm(){

    using namespace std;

    lista = new ListaOrdenada ();

    conversion = new Natura_Interval_extension ();
    lista = new ListaOrdenada ();
    deteccion = new DeteccionViolacionBoundaries();

    anterior_sis_min = new QVector <qreal> ();

    mejor_k = std::numeric_limits<qreal>::infinity();
    depuracion = true;

    plantas_nominales = new QVector <cxsc::complex> ();
    plantas_nominales2 = new QVector <std::complex <qreal>> ();

    foreach (qreal o, *omega) {
        std::complex <qreal> c = planta->getPunto(o);
        plantas_nominales2->append(c);
        plantas_nominales->append(cxsc::complex(c.real(), c.imag()));
    }


    comprobarVariables(controlador);

    check_box_feasibility(controlador);

    while (true){

        if (lista->isVacia()){
            menerror("El espacio de parámetros inicial del controlador no es válido.", "Loop Shaping");

            delete conversion;
            delete lista;
            delete anterior_sis_min;
            delete controlador_inicial;
            delete deteccion;

            return false;
        }

        Tripleta * tripleta = lista->recuperar();
        lista->borrarPrimero();

        //local_optimization(tripleta->sistema);

        //if (tripleta->k <= mejor_k){

        if (tripleta->flags == feasible || if_less_epsilon(tripleta->sistema, this->epsilon, omega, conversion, plantas_nominales)){
            if (tripleta->flags == ambiguous){
                controlador_retorno = guardarControlador(tripleta->sistema, false);
            } else {
                controlador_retorno = guardarControlador(tripleta->sistema, true);
            }

            delete conversion;
            delete lista;
            delete tripleta;
            delete anterior_sis_min;
            delete controlador_inicial;
            delete deteccion;

            return true;
        }

        /*cout << "k: [" + QString::number(tripleta->sistema->getK()->getRango().x()).toStdString() + ", " +
                QString::number(tripleta->sistema->getK()->getRango().y()).toStdString() + "] " << endl;

        cout << "nume: [" + QString::number(tripleta->sistema->getNumerador()->at(0)->getRango().x()).toStdString() + ", " +
                QString::number(tripleta->sistema->getNumerador()->at(0)->getRango().y()).toStdString() + "] " << endl;

        cout << "............................................................................" << endl;*/

        /* flags_box a = tripleta->flags;

                if (a == ambiguous){
                    cout << "ambiguous" << endl;
                }else if (a == feasible){
                    cout << "feasible" << endl;
                } else {
                    cout << "infeasible" << endl;
                }*/

        //Split blox
        struct return_bisection retur = split_box_bisection(tripleta->sistema);

        delete tripleta;

        if (!retur.descartado){
            check_box_feasibility(retur.v1);
            check_box_feasibility(retur.v2);
        }
        //} //cierre mejor k
    }


    return true;
}


//Función que retorna el controlador.
Sistema * Algorithm_nandkishor::getControlador(){
    return controlador_retorno;
}


//Función que comprueba si la caja actual es feasible, infeasible o ambiguous.
inline flags_box Algorithm_nandkishor::check_box_feasibility(Sistema * controlador){

    using namespace std;

    struct datos_caja datos;
    flags_box flag_final = feasible;

#ifdef DEBUG
    QVector <QVector <QPointF> * > * puntos;

    if (depuracion)
        puntos = new QVector <QVector <QPointF> * > ();
#endif

    QVector <datos_caja> * datosCortesBoundaries = new QVector <datos_caja> ();
    bool penalizacion = false;
    cinterval caja;

    bool completo = false;


    for (qint32 k = 0; k < omega->size(); k++) {


        caja = conversion->get_box(controlador, omega->at(k), plantas_nominales->at(k), false);

        datos = deteccion->deteccionViolacionCajaNi(caja, boundaries, k);

#ifdef DEBUG
        if (depuracion)
            puntos->append(deteccion_violacion->getPuntos());
#endif


        if (datos.flag == ambiguous){
            flag_final = ambiguous;
        } else if (datos.flag == infeasible){
            delete controlador;
            datosCortesBoundaries->clear();
            return infeasible;
        }

        datosCortesBoundaries->append(datos);

        if (omega->at(k) == 2 && SupIm(caja) < -180){
            penalizacion = true;
        }

        if (datos.completo){
            completo = true;
        }

    }

    if (flag_final == ambiguous && !completo){
        controlador = acelerated(controlador, datosCortesBoundaries);
    }


    datosCortesBoundaries->clear();


#ifdef DEBUG
    //if (depuracion)
    //mostrar_diagrama(puntos);
#endif

    lista->insertar(new Tripleta(penalizacion ? controlador->getK()->getRango().x() + 100 : controlador->getK()->getRango().x(), controlador, flag_final));

    return flag_final;
}

//Función que recorta la caja.
inline Sistema * Algorithm_nandkishor::acelerated(Sistema * v, QVector<datos_caja> *datosCortesBoundaries) {

    QVector <Var *> * denominador = v->getDenominador();
    QVector <Var *> * numerador = v->getNumerador();
    QPointF k = v->getK()->getRango();
    QPointF kNuevo;


    kNuevo.setX(20 * log10(v->getK()->getRango().x()));
    kNuevo.setY(20 * log10(v->getK()->getRango().y()));

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

        if (datosCortesBoundaries->at(i).flag == ambiguous && !datosCortesBoundaries->at(i).completo) {

            o = omega->at(i);
            plantaNominal = plantas_nominales2->at(i);
            cortesMin = datosCortesBoundaries->at(i).minimosMaximos->at(0);
            cortesMax = datosCortesBoundaries->at(i).minimosMaximos->at(1);

            //Análisis de la ganancia

            if (datosCortesBoundaries->at(i).recAbajo && datosCortesBoundaries->at(i).uniAbajo){
                nuevoMinKReal = cortesMin / 20 * log10(abs(v->getPunto(numeradorSup, denominadorInf, 1, 0, o) * plantaNominal));

                if (nuevoMinKReal > kNuevo.x() && nuevoMinKReal < kNuevo.y()) {
                    kNuevo.setX(nuevoMinKReal);
                }
            }else if (datosCortesBoundaries->at(i).recArriba && datosCortesBoundaries->at(i).uniArriba) {
                nuevoMaxKReal = cortesMax / 20 * log10(abs(v->getPunto(numeradorInf, denominadorSup, 1, 0, o) * plantaNominal));

                if (nuevoMaxKReal > kNuevo.x() && nuevoMaxKReal < kNuevo.y()) {
                    kNuevo.setY(nuevoMaxKReal);
                }
            }

            //Numerador
            if (isVariableNume){
                for (qint32 j = 0; j < numerador->size(); j++) {
                    if (numerador->at(j)->isVariable()){
                        if (datosCortesBoundaries->at(i).recAbajo && datosCortesBoundaries->at(i).uniAbajo){
                            n = numeradorSup->at(j);

                            numeradorSup->remove(j);

                            nuevoMinNume = sqrt( pow((cortesMin * 20 * log10(abs (v->getPuntoDeno(denominadorInf, o)))) /
                                                     (20 * log10(k.y()) *  20 * log10(abs (v->getPuntoNume(numeradorSup, o) * plantaNominal))), 2) - 20 * log10(pow(o, 2)));


                            numeradorSup->insert(j, n);

                            if (nuevoMinNume > numeradorInfNuevo->at(j) && nuevoMinNume < numeradorSupNuevo->at(j)) {
                                numeradorInfNuevo->replace(j, nuevoMinNume);
                            }
                        } else if (datosCortesBoundaries->at(i).recArriba && datosCortesBoundaries->at(i).uniArriba) {
                            n = numeradorInf->at(j);

                            numeradorInf->remove(j);

                            nuevoMaxNume = sqrt( pow((cortesMax * 20 * log10(abs (v->getPuntoDeno(denominadorSup, o)))) /
                                                     (20 * log10(k.x()) *  20 * log10(abs (v->getPuntoNume(numeradorInf, o) * plantaNominal))), 2) - 20 * log10(pow(o, 2)));


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

                        if (datosCortesBoundaries->at(i).recAbajo && datosCortesBoundaries->at(i).uniAbajo) {
                            n = denominadorInf->at(j);

                            denominadorInf->remove(j);

                            nuevoMaxDeno = sqrt(pow((20 * log10(k.y()) * 20 * log10(abs (v->getPuntoNume(numeradorSup, o) * plantaNominal))) /
                                                    (cortesMin * 20 * log10(abs(v->getPuntoDeno(denominadorInf, o)))), 2) - 20 * log10(pow(o, 2)));

                            denominadorInf->insert(j, n);

                            if (nuevoMaxDeno < denominadorSupNuevo->at(j) && nuevoMaxDeno > denominadorInfNuevo->at(j)) {
                                denominadorSupNuevo->replace(j, nuevoMaxDeno);
                            }
                        } else  if (datosCortesBoundaries->at(i).recArriba && datosCortesBoundaries->at(i).uniArriba) {
                            n = denominadorSup->at(j);

                            denominadorSup->remove(j);

                            nuevoMinDeno = sqrt(pow((20 * log10(k.x()) * 20 * log10(abs (v->getPuntoNume(numeradorInf, o) * plantaNominal))) /
                                                    (cortesMax * 20 * log10(abs(v->getPuntoDeno(denominadorSup, o)))), 2) - 20 * log10(pow(o, 2)));

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
            var_nume_nuevo = new Var("", QPointF(pow(10, numeradorInfNuevo->at(i) / 20), pow(10, numeradorSupNuevo->at(i) / 20)), 0);
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
            var_deno_nuevo = new Var("", QPointF(pow(10, denominadorInfNuevo->at(i) / 20), pow(10, denominadorSupNuevo->at(i) / 20)), 0);
        } else {
            var_deno_nuevo = new Var(var_deno_antiguo->getNominal());
        }

        denominador_nuevo->append(var_deno_nuevo);
    }

    kNuevo.setX(pow(10, kNuevo.x() / 20));
    kNuevo.setY(pow(10, kNuevo.y() / 20));

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

}


//Función que retorna la mejor k.
/*inline qreal Algorithm_nandkishor::get_k(Sistema *controlador, QVector <qreal> * nume_sup, QVector <qreal> * deno_inf,
                                         qreal minimo_boundarie,
                                         std::complex<qreal> p0, qreal omega, qreal k_min, qreal k_max){
    qreal k_nueva_lineal = minimo_boundarie / abs (controlador->getPunto(nume_sup, deno_inf, 1, 0, omega) * p0);

    if (k_nueva_lineal > k_min && k_nueva_lineal < k_max){

        //cout << "K: " << k_min << " nueva k: " << k_nueva_lineal << endl;
        return k_nueva_lineal;
    }

    return k_min;
}



//Función que retorna el mejor numerador.
inline QVector<qreal> * Algorithm_nandkishor::get_nume_kganancia(Sistema *controlador, QVector <qreal> * nume_sup, QVector <qreal> * deno_inf,
                                                                 qreal k_max,
                                                                 qreal minimo_boundarie, std::complex<qreal> p0, qreal omega,
                                                                 QVector <qreal> * nume_inf){

    QVector <Var *> * numerador = controlador->getNumerador();

    for (qint32 i = 0; i < nume_sup->size(); i++){

        if (numerador->at(i)->isVariable()){

            qreal n = nume_sup->at(i);

            nume_sup->remove(i);

            qreal z = sqrt( pow((minimo_boundarie * abs (controlador->getPuntoDeno(deno_inf, omega))) /
                                (k_max *  abs (controlador->getPuntoNume(nume_sup, omega) * p0)), 2) - pow(omega, 2));

            nume_sup->insert(i, n);

            if (z > nume_inf->at(i) && z < n){
                //cout << "nume: " << nume_inf->at(i) << " nuevo nume: " << z << endl;
                nume_inf->replace(i, z);
            }
        }
    }

    return nume_inf;
}

//Función que retorna el mejor denominador.
inline QVector<qreal> * Algorithm_nandkishor::get_deno_kganancia(Sistema *controlador, QVector <qreal> * nume_sup, QVector <qreal> * deno_inf,
                                                                 qreal k_max, qreal minimo_boundarie, std::complex<qreal> p0, qreal omega, QVector <qreal> * deno_sup){

    QVector <Var *> * denominador = controlador->getDenominador();

    for (qint32 i = 0; i < deno_inf->size(); i++){

        if (denominador->at(i)->isVariable()){

            qreal n = deno_inf->at(i);

            deno_inf->remove(i);

            qreal z = sqrt(pow((k_max * abs (controlador->getPuntoNume(nume_sup, omega) * p0)) /
                               (minimo_boundarie * abs(controlador->getPuntoDeno(deno_inf, omega))), 2) - pow(omega, 2));

            deno_inf->insert(i, n);

            if (z > n && z < deno_sup->at(i)){
                //cout << "deno: " << n << " nuevo deno: " << z << endl;
                deno_sup->replace(i, z);
            }
        }
    }

    return deno_sup;
}

//Función que retorna la mejor k.
inline qreal Algorithm_nandkishor::get_k_max(Sistema *controlador, QVector <qreal> * nume_inf, QVector <qreal> * deno_sup,
                                             qreal maximo_boundarie,
                                             std::complex<qreal> p0, qreal omega, qreal k_min, qreal k_max){

    qreal k_nueva_lineal = maximo_boundarie / abs (controlador->getPunto(nume_inf, deno_sup, 1, 0, omega) * p0);

    if (k_nueva_lineal > k_min && k_nueva_lineal < k_max){

        //cout << "K: " << k_max << " nueva k: " << k_nueva_lineal << endl;
        return k_nueva_lineal;
    }

    return k_max;
}



//Función que retorna el mejor numerador.
inline QVector<qreal> * Algorithm_nandkishor::get_nume_kganancia_max(Sistema *controlador, QVector <qreal> * nume_inf, QVector <qreal> * deno_sup,
                                                                     qreal k_min,
                                                                     qreal maximo_boundarie, std::complex<qreal> p0, qreal omega,
                                                                     QVector <qreal> * nume_sup){

    QVector <Var *> * numerador = controlador->getNumerador();

    for (qint32 i = 0; i < nume_inf->size(); i++){

        if (numerador->at(i)->isVariable()){

            qreal n = nume_inf->at(i);

            nume_inf->remove(i);

            qreal z = sqrt( pow((maximo_boundarie * abs (controlador->getPuntoDeno(deno_sup, omega))) /
                                (k_min *  abs (controlador->getPuntoNume(nume_inf, omega) * p0)), 2) - pow(omega, 2));

            nume_inf->insert(i, n);

            if (z < nume_sup->at(i) && z > n){
                //cout << "nume: " << nume_inf->at(i) << " nuevo nume: " << z << endl;
                nume_sup->replace(i, z);
            }
        }
    }

    return nume_sup;
}

//Función que retorna el mejor denominador.
inline QVector<qreal> * Algorithm_nandkishor::get_deno_kganancia_max(Sistema *controlador, QVector <qreal> * nume_inf, QVector <qreal> * deno_sup,
                                                                     qreal k_min, qreal maximo_boundarie, std::complex<qreal> p0, qreal omega, QVector <qreal> * deno_inf){

    QVector <Var *> * denominador = controlador->getDenominador();

    for (qint32 i = 0; i < deno_sup->size(); i++){

        if (denominador->at(i)->isVariable()){

            qreal n = deno_sup->at(i);

            deno_sup->remove(i);

            qreal z = sqrt(pow((k_min * abs (controlador->getPuntoNume(nume_inf, omega) * p0)) /
                               (maximo_boundarie * abs(controlador->getPuntoDeno(deno_sup, omega))), 2) - pow(omega, 2));

            deno_sup->insert(i, n);

            if (z < n && z > deno_inf->at(i)){
                //cout << "deno: " << n << " nuevo deno: " << z << endl;
                deno_inf->replace(i, z);
            }
        }
    }

    return deno_inf;
}

inline QVector<qreal> * Algorithm_nandkishor::get_nume_knganancia(Sistema *controlador, QVector<qreal> *nume, QVector<qreal> *deno,
                                                                  qreal k, qreal minimo_boundarie, std::complex<qreal> p0,
                                                                  qreal omega, QVector<qreal> *nume_bajo){

    QVector <Var *> * numerador = controlador->getNumerador();


    for (qint32 i = 0; i < nume->size(); i++){

        if (numerador->at(i)->isVariable()){

            qreal n = nume->at(i);

            nume->remove(i);

            qreal z = sqrt( (pow(omega, 2)) / (abs(pow((minimo_boundarie * controlador->getPuntoDeno(deno, omega))
                                                       / (k * controlador->getPuntoNume(nume, omega) * p0), 2)) - 1.0));

            nume->insert(i, n);

            if (z > nume_bajo->at(i)){
                //cout << "nume: " << z - nume_bajo->at(i) << endl;
                nume_bajo->replace(i, z);
            }
        }
    }

    return nume_bajo;
}

//Función que retorna el mejor denominador.
inline QVector<qreal> * Algorithm_nandkishor::get_deno_knganancia(Sistema *controlador, QVector <qreal> * nume, QVector <qreal> * deno_inf, QVector <qreal> * deno_sup,
                                                                  qreal k,
                                                                  qreal minimo_boundarie, std::complex<qreal> p0, qreal omega){

    QVector <Var *> * denominador = controlador->getDenominador();

    for (qint32 i = 0; i < deno_inf->size(); i++){

        if (denominador->at(i)->isVariable()){

            qreal n = deno_inf->at(i);
            deno_inf->remove(i);

            qreal z =  sqrt( (pow(omega, 2)) / (abs(pow( (k * controlador->getPuntoNume(nume, omega) * p0) /
                                                         (minimo_boundarie * controlador->getPuntoDeno(deno_inf, omega)) , 2)) - 1.0));

            deno_inf->insert(i, n);

            if (z > n){
                //cout << "deno: " << z - n << endl;
                deno_sup->replace(i, z);
            }
        }
    }

    return deno_sup;
}*/

//Función que hace la búsqueda local.
inline void Algorithm_nandkishor::local_optimization(Sistema *controlador){

    qreal nuevo_min = 0;

    nuevo_min += pow(controlador->getK()->getNominal(), 2);

    foreach (Var * var, *controlador->getNumerador()) {
        nuevo_min += var->getRango().x();
    }


    foreach (Var * var, *controlador->getDenominador()) {
        nuevo_min += var->getRango().x();
    }

    nuevo_min = abs(sqrt(nuevo_min));

    bool hacer = true;

    foreach (qreal sis_min, *anterior_sis_min) {
        if ((nuevo_min - sis_min) < 0.1){
            hacer = false;
        }
    }

    if (hacer){

        anterior_sis_min->append(nuevo_min);

        //std::cout << "Búsqueda local" << std::endl;
        qreal nueva_mejor_k = busqueda_local(delta, controlador);

        if (nueva_mejor_k < mejor_k){
            mejor_k = nueva_mejor_k;
            //std::cout << "Mejor k: " << mejor_k << std::endl;
        }
    }
}

//Función que checkea si el sistema viola o no los boundaries para el algoritmo de búsqueda local.
inline flags_box Algorithm_nandkishor::check_box_feasibility(QVector<qreal> *nume, QVector<qreal> *deno, qreal k,
                                                             qreal ret){

    using namespace std;

    struct datos_caja datos;

    flags_box flag_final = feasible;

    cinterval box;
    qint32 contador = 0;

    foreach (qreal o, *omega) {

        std::complex <qreal> c = planta->getPunto(o) * controlador_inicial->getPunto(nume, deno, k, ret, o);
        box = cinterval (interval(c.real()), interval(c.imag()));

        datos = deteccion->deteccionViolacionCajaNiNi(box, boundaries, k);


        if (datos.flag == ambiguous){
            flag_final = ambiguous;
        } else if (datos.flag == infeasible){
            datos.minimosMaximos->clear();
            return infeasible;
        }

        contador++;
    }

    return flag_final;
}

inline void Algorithm_nandkishor::comprobarVariables(Sistema *controlador){
    bool b = true;

    foreach (Var * var, *controlador->getNumerador()) {
        if (var->isVariable()){
            b = false;
        }
    }

    isVariableNume = !b;

    b = true;

    foreach (Var * var, *controlador->getDenominador()) {
        if (var->isVariable()){
            b = false;
        }
    }

    isVariableDeno = !b;
}

inline qint32 Algorithm_nandkishor::crearVectores(Sistema *controlador, QVector<qreal> *numerador, QVector<qreal> *denominador,
                                                  QVector<qreal> * k,
                                                  QVector<QVector<qreal> *> *variables, qreal delta,
                                                  QVector <qreal> * numeNominales,
                                                  QVector <qreal> * denoNominales, qreal kNominal){

    QVector <Var *> * nume = controlador->getNumerador();
    QVector <Var *> * deno = controlador->getDenominador();

    qint32 lonNume = nume->size();

    qint32 lonDeno = deno->size();

    qint32 combinaciones = 1;
    qint32 i;

    for (i = 0; i < lonNume; i++){
        Var * n = nume->at(i);
        qreal nominal = numeNominales->at(i);
        QVector <qreal> * vector = new QVector <qreal> ();
        qint32 mult = 0;
        if (n->isVariable()){

            if ((nominal-delta) > n->getRango().x()){
                vector->append(nominal - delta);
                mult++;
            }

            vector->append(nominal);
            mult++;

            if ((nominal+delta) < n->getRango().y()){
                vector->append(nominal + delta);
                mult++;
            }

        } else{
            vector->append(nominal);
            mult++;
        }


        combinaciones *= mult;
        variables->insert(i,vector);
        numerador->insert(i,vector->at(0));
    }

    qint32 salto = i;

    for (i = 0; i < lonDeno; i++){
        Var * n = deno->at(i);
        qreal nominal = denoNominales->at(i);
        QVector <qreal> * vector = new QVector <qreal> ();
        qint32 mult = 0;
        if (n->isVariable()){

            if ((nominal-delta) > n->getRango().x()){
                vector->append(nominal - delta);
                mult++;
            }

            vector->append(nominal);
            mult++;

            if ((nominal+delta) < n->getRango().y()){
                vector->append(nominal + delta);
                mult++;
            }

        } else{
            vector->append(nominal);
            mult++;
        }


        combinaciones *= mult;
        variables->insert(salto + i,vector);
        denominador->insert(i,vector->at(0));
    }

    Var * n = controlador->getK();
    qreal nominal = kNominal;
    if (n->isVariable()){

        if ((nominal-delta) > n->getRango().x()){
            k->append(nominal - delta);
        }

        k->append(nominal);

        if ((nominal+delta) < n->getRango().y()){
            k->append(nominal + delta);
        }

    }else{
        k->append(controlador->getK()->getNominal());
    }


    return combinaciones;
}

inline qreal Algorithm_nandkishor::inicializacion(Sistema * controlador, QVector<qreal> *numerador, QVector<qreal> *denominador,
                                                  tipoInicializacion tipo){

    if (tipo == centro){
        foreach (Var * var, *controlador->getNumerador()) {
            numerador->append(var->getNominal());
        }

        foreach (Var * var, *controlador->getDenominador()) {
            denominador->append(var->getNominal());
        }

        return controlador->getK()->getNominal();
    } else if (tipo == aleatorio){
        foreach (Var * var, *controlador->getNumerador()) {
            QPointF r = var->getRango();
            numerador->append(r.x() + (qrand() % (qint32) (r.y() - r.x() + 1)));
        }

        foreach (Var * var, *controlador->getDenominador()) {
            QPointF r = var->getRango();
            denominador->append(r.x() + (qrand() % (qint32) (r.y() - r.x() + 1)));
        }

        QPointF  k = controlador->getK()->getRango();

        qreal a = k.x() + (qrand() % (qint32) (k.y() - k.x() + 1));


        return a;
    } else {
        foreach (Var * var, *controlador->getNumerador()) {
            QPointF r = var->getRango();
            numerador->append(r.x());
        }

        foreach (Var * var, *controlador->getDenominador()) {
            QPointF r = var->getRango();
            denominador->append(r.y());
        }

        return controlador->getK()->getRango().y();
    }

    return 0;
}

//Algorítmo de búsqueda local.
inline qreal Algorithm_nandkishor::busqueda_local(qreal delta, Sistema * controlador){

    QVector <Var *> * nume = controlador->getNumerador();
    QVector <Var *> * deno = controlador->getDenominador();

    QVector <qreal> * numeNominales = new QVector <qreal> ();
    QVector <qreal> * denoNominales = new QVector <qreal> ();

    qreal kNominales = inicializacion(controlador, numeNominales, denoNominales, ini);

    //Declarvamos las variables
    qint32 lonNume = nume->size();
    qint32 lonDeno = deno->size();

    QVector <qreal> * denominador = new QVector <qreal> ();
    QVector <qreal> * numerador = new QVector <qreal> ();
    numerador->reserve(lonNume);
    denominador->reserve(lonDeno);

    QVector<QVector<qreal> * > * variables = new QVector <QVector <qreal> * > ();
    variables->reserve(lonNume + lonDeno);

    QVector <qreal> * k = new QVector <qreal> ();

    bool resultado_encontrado = false;
    qreal mejor_k = std::numeric_limits<qreal>::infinity();
    qreal mejor_k_global = std::numeric_limits<qreal>::infinity();

    qint32 combinaciones = crearVectores(controlador, numerador, denominador, k, variables,delta, numeNominales,
                                         denoNominales, kNominales);


    qreal retT = 0;

    while (!resultado_encontrado){

        QVector <qint32> * contador = new QVector <qint32> (lonDeno + lonNume + 1, 0);

        for (qint32 i = 0; i < combinaciones; i++){

            foreach (qreal kv, *k) {

                if (check_box_feasibility(numerador, denominador, kv, retT) == feasible){

                    if (kv < mejor_k){
                        mejor_k = kv;

                        numeNominales->clear();
                        denoNominales->clear();

                        numeNominales = new QVector <qreal> (*numerador);
                        denoNominales = new QVector <qreal> (*denominador);
                        kNominales = kv;
                    }
                }
            }

            contador->replace(0, contador->first()+1);

            bool salir = false;

            for (qint32 j = 0; j < lonNume+lonDeno && salir == false;j++){

                if (contador->at(j) >= (variables->at(j)->size())){
                    contador->replace(j,0);
                    contador->replace(j+1, contador->at(j+1) +1);
                }else {
                    salir = true;
                }

                if (j < lonNume){
                    numerador->replace(j,variables->at(j)->at(contador->at(j)));

                }else {
                    denominador->replace(j-lonNume,variables->at(j)->at(contador->at(j)));

                }
            }
        }

        numerador->clear();
        denominador->clear();
        contador->clear();
        variables->clear();
        k->clear();

        denominador = new QVector <qreal> ();
        numerador = new QVector <qreal> ();
        numerador->reserve(lonNume);
        denominador->reserve(lonDeno);

        variables = new QVector <QVector <qreal> * > ();
        variables->reserve(lonNume + lonDeno);

        k = new QVector <qreal> ();

        if (mejor_k < mejor_k_global){
            mejor_k_global = mejor_k;
            resultado_encontrado = false;
        } else {
            resultado_encontrado = true;
        }

        crearVectores(controlador, numerador, denominador, k, variables,delta, numeNominales, denoNominales, kNominales);
    }

    numeNominales->clear();
    denoNominales->clear();
    denominador->clear();
    numerador->clear();
    variables->clear();

    return mejor_k;
}

inline qreal Algorithm_nandkishor::log10(qreal a){
    if (a == 0){
        a = 0.001;
    }

    return std::log10(a);
}


/*cout << "k: [" + QString::number(tripleta->sistema->getK()->getRango().x()).toStdString() + ", " +
        QString::number(tripleta->sistema->getK()->getRango().y()).toStdString() + "]        ------------          ";

//cout << "mejor K: " + QString::number(mejor_k).toStdString() << endl;

//flags_box a = tripleta->flags;


cout << "nume: " << QString::number(tripleta->sistema->getNumerador()->at(0)->getRango().x()).toStdString() << ", " <<
        QString::number(tripleta->sistema->getNumerador()->at(0)->getRango().y()).toStdString() << endl;


cout << "deno: " << QString::number(tripleta->sistema->getDenominador()->at(0)->getRango().x()).toStdString() << ", " <<
        QString::number(tripleta->sistema->getDenominador()->at(0)->getRango().y()).toStdString() << endl << endl;

if (tripleta->flags == ambiguous){
cout << "ambiguous" << endl;
}else if (tripleta->flags == feasible){
cout << "feasible" << endl;
} else {
cout << "infeasible" << endl;
}*/
