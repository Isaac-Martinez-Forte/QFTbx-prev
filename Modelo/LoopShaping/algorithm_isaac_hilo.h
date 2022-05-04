#ifndef ALGORITHM_ISAAC_HILO_H
#define ALGORITHM_ISAAC_HILO_H

#include "funcionescomunes.h"
#include "Modelo/LoopShaping/algorithm_isaac.h"

class Algorithm_isaac;

class Algorithm_isaac_hilo : public  QThread
{
    Q_OBJECT

public:
    Algorithm_isaac_hilo(Algorithm_isaac * funciones);

    void run();

    friend class Algorithm_isaac;

private:

    Algorithm_isaac * funciones;
};

#endif

