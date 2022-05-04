#ifndef LISTAORDENADA_H
#define LISTAORDENADA_H

#include <QLinkedList>
#include "tripleta.h"

class ListaOrdenada
{
public:
    ListaOrdenada();
    ~ListaOrdenada();

    void insertar (Tripleta * elemento);

    Tripleta * recuperar ();

    void borrarPrimero();

    bool isVacia ();

private:

    struct Lista
    {
        Tripleta * elemento;
        Lista * sig;
    };

    Lista * l;

};

#endif // LISTAORDENADA_H
