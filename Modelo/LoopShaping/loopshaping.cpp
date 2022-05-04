#include "loopshaping.h"

using namespace std;


//#define pruebas
//#define saltar

LoopShaping::LoopShaping()
{

}

LoopShaping::~LoopShaping()
{

}


bool LoopShaping::iniciar(Sistema *planta, Sistema *controlador, QVector<qreal> *omega, DatosBound *boundaries,
                          qreal epsilon, tools::alg_loop_shaping seleccionado, bool depuracion, qreal delta,
                          QVector <QVector <std::complex <qreal> > * > * temp, QVector <tools::dBND *> * espe,
                          qint32 inicializacion, bool hilos, bool bisection_avanced, bool deteccion_avanced, bool a){


    //sacamos el círculo envolvente del boundarie
    QVector< QVector<QPointF> * > * boun = boundaries->getBoundariesReun();

    qreal maglineal;
    qreal x,y;

    qreal mayorDistancia, menorDistancia;
    qreal distancia;

    qreal media_x, media_y;
    QVector <QPointF> * centros = new QVector <QPointF> ();
    QVector <qreal> * radiosMayor = new QVector <qreal> ();
    QVector <qreal> * radiosMenor = new QVector <qreal> ();

    foreach (auto vector, *boun) {

        mayorDistancia = std::numeric_limits<qreal>::lowest();
        menorDistancia = std::numeric_limits<qreal>::max();
        media_x = 0; media_y = 0;

        foreach (auto p, *vector) {
            maglineal = pow(10,p.y()/20);
            media_x += maglineal * cos (p.x() * M_PI / 180);
            media_y += maglineal * sin (p.x() * M_PI / 180);
        }

        QPointF centro (media_x/vector->size(), media_y/vector->size());

        foreach (auto p, *vector) {
            maglineal = pow(10,p.y()/20);
            x = maglineal * cos (p.x() * M_PI / 180);
            y = maglineal * sin (p.x() * M_PI / 180);

            distancia = sqrt(pow(x - centro.x(), 2) + pow(y - centro.y(), 2));

            if (distancia > mayorDistancia){
                mayorDistancia = distancia;
            }

            if (distancia < menorDistancia){
                menorDistancia = distancia;
            }
        }

        centros->append(centro);
        radiosMayor->append(mayorDistancia);
        radiosMenor->append(menorDistancia);
    }

    /*QVector< QVector< QVector<QPointF> * > * > * nuevo_boundaries_hash = new QVector < QVector <QVector <QPointF > * > * > ();
      QVector< QVector< QVector<QPointF> * > * > * boundaries_hash = boundaries->getBoundariesReunHash();
    QVector <qint32> * n_posiciones = new QVector <qint32> ();

    qint32 contador;
    qreal anterior_x = std::numeric_limits<qreal>::lowest();
    qreal tam = abs (boundaries->getDatosFas().x() - boundaries->getDatosFas().y()) / boundaries->getTamFas();
    QVector<QPointF> * nuevo_vector;

    foreach (auto hash, *boundaries_hash) {
        QVector< QVector<QPointF> * > * nuevo_hash = new QVector <QVector <QPointF> * > ();
        contador = 0;
        foreach (auto vector, *hash) {
            foreach (QPointF punto, *vector) {
                if (abs(punto.x() + anterior_x) > tam){
                    nuevo_hash->append(nuevo_vector);
                    nuevo_vector = new QVector <QPointF> ();
                    nuevo_vector->append(punto);
                    contador++;
                }else{
                    nuevo_vector->append(punto);
                }
            }
        }
        n_posiciones->append(contador);
        nuevo_hash->removeFirst();
        nuevo_boundaries_hash->append(nuevo_hash);
    }*/


    //Guardamos la granularidad
    foreach (Var * v, *controlador->getNumerador()) {
        if (v->isVariable())
            v->setGranularidadFormula(epsilon);
    }

    foreach (Var * v, *controlador->getDenominador()) {
        if (v->isVariable())
            v->setGranularidadFormula(epsilon);
    }

    controlador->getK()->setGranularidadFormula(epsilon);


    QElapsedTimer timer;

    bool re;

#ifdef pruebas
    qint32 nVariables = 8, contador = 1, contadorNume = 0, contadorDeno = 0;

    if (seleccionado == tools::sachin){
        std::cout << "Algoritmo Sachín" << std::endl;
    }else if (seleccionado == tools::nandkishor){
        std::cout << "Algoritmo Nandkishor" << std::endl;
    } else if(seleccionado == tools::isaac){
        std::cout << "Algoritmo Isaac" << std::endl;
    }


    for (; contador <= nVariables;) {
#endif

#ifdef saltar
        if (contador != 5) {
#endif

            if (seleccionado == tools::sachin){
#ifndef pruebas
                std::cout << "Algoritmo Sachín" << std::endl;
#endif
                Algorithm_sachin * sachin = new Algorithm_sachin();
                timer.start();
                sachin->set_datos(planta, controlador, omega, boundaries, epsilon, boundaries->getBoundariesReunHash());
                re =  sachin->init_algorithm();
#ifndef pruebas
                if (re) {
                    this->controlador = sachin->getControlador();
                    std::cout << "LoopShaping: " << timer.elapsed() << " milliseconds" << std::endl;
                    std::cout << "k: " << this->controlador->getK()->getRango().x() << std::endl;
                }


                delete sachin;

                return re;
#else
                std::cout << timer.elapsed() << std::endl;
                delete sachin;
#endif
            } else if (seleccionado == tools::nandkishor){
#ifndef pruebas
                std::cout << "Algoritmo Nandkishor" << std::endl;
#endif
                timer.start();
                Algorithm_nandkishor * nandkishor = new Algorithm_nandkishor();
                nandkishor->set_datos(planta, controlador, omega, boundaries, epsilon, boundaries->getBoundariesReunHash(),
                                      delta, inicializacion);
                re =  nandkishor->init_algorithm();

#ifndef pruebas
                if (re){
                    this->controlador = nandkishor->getControlador();

                    std::cout << "LoopShaping: " << timer.elapsed() << " milliseconds" << std::endl;
                    std::cout << "k: " << this->controlador->getK()->getRango().x() << std::endl;

                }

                delete nandkishor;

                return re;
#else
                std::cout << timer.elapsed()  << std::endl;
                delete nandkishor;
#endif
            } else if (seleccionado == tools::rambabu){

                std::cout << "Algoritmo Rambabú" << std::endl;

                Algorithm_rambabu * rambabu = new Algorithm_rambabu();
                rambabu->set_datos(planta, controlador, omega, boundaries, epsilon, boundaries->getBoundariesReunHash(),
                                   depuracion, temp, espe);
                re =  rambabu->init_algorithm();

                if(re){
                    this->controlador = rambabu->getControlador();

                    std::cout << "LoopShaping: " << timer.elapsed() << " milliseconds" << std::endl;
                    std::cout << "k: " << this->controlador->getK()->getRango().x() << std::endl;

                }

                delete rambabu;

                return re;
            } else if(seleccionado == tools::isaac){
#ifndef pruebas
                std::cout << "Algoritmo Isaac" << std::endl;
#endif
                Algorithm_isaac * isaac = new Algorithm_isaac();

                isaac->set_datos(planta, controlador, omega, boundaries, epsilon, boundaries->getBoundariesReunHash(),
                                 depuracion, hilos, radiosMayor, radiosMenor, centros, bisection_avanced, deteccion_avanced, a);

                timer.start();

                re =  isaac->init_algorithm();
#ifndef pruebas

                if (re) {
                    std::cout << "LoopShaping: " << timer.elapsed() << " milliseconds" << std::endl;
                    this->controlador = isaac->getControlador();
                    std::cout << "k: " << this->controlador->getK()->getRango().x() << std::endl;
                }

                delete isaac;

                return re;
#else
                std::cout << timer.elapsed() << std::endl;
                delete isaac;
#endif
            }
#ifdef pruebas


#ifdef saltar
        }
#endif

        contador++;

        if (contador % 2 == 0) {
            controlador->getDenominador()->at(contadorDeno)->setVariable(true);
            contadorDeno++;
        } else {

            controlador->getNumerador()->at(contadorNume)->setVariable(true);
            contadorNume++;
        }
    }

    std::cout << "terminado" << std::endl;

    return re;
#endif

    return false;
}


Sistema * LoopShaping::getControlador(){
    return controlador;
}
