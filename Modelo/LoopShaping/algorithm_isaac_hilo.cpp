#include "algorithm_isaac_hilo.h"

using namespace FC;

Algorithm_isaac_hilo::Algorithm_isaac_hilo(Algorithm_isaac * funciones){
    this->funciones = funciones;
}

void Algorithm_isaac_hilo::run(){

    Tripleta * tripleta;
    struct return_bisection retur;

    while (true){

        /*if (lista->isVacia()){
            menerror("El espacio de parámetros inicial del controlador no es válido.", "Loop Shaping");

            emit resultGo(false);
        }*/


        //Exclusión mutua para acceder a la lista.
        funciones->semaforo->acquire();
        funciones->mutexAccesoLista.lock();
        tripleta = funciones->lista->recuperar();
        funciones->lista->borrarPrimero();
        funciones->mutexAccesoLista.unlock();

        if (if_less_epsilon(tripleta->sistema, funciones->epsilon, funciones->omega, funciones->conversion, funciones->plantas_nominales)){
            funciones->mutexEnding.lock();
            if (tripleta->flags == ambiguous){
                funciones->controlador_retorno = guardarControlador(tripleta->sistema, false);
            } else {
                funciones->controlador_retorno = guardarControlador(tripleta->sistema, true);
            }

            funciones->terminacionCorrecta = true;
            funciones->mutexTerminar.unlock();

            return;
        }

        //Split blox
        retur = split_box_bisection(tripleta->sistema);

        delete tripleta;

        if (!retur.descartado){
            /*t = funciones->check_box_feasibility(retur.v1);
            if (t!=NULL){
                funciones->mutexAccesoLista.lock();
                funciones->lista->insertar(t);
                funciones->mutexAccesoLista.unlock();
                funciones->semaforo->release();
            }

            t = funciones->check_box_feasibility(retur.v2);
            if (t!=NULL){
                funciones->mutexAccesoLista.lock();
                funciones->lista->insertar(t);
                funciones->mutexAccesoLista.unlock();
                funciones->semaforo->release();
            }*/
        }
    }
}
