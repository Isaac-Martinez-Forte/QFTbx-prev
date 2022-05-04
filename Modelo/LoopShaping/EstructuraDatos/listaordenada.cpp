#include "listaordenada.h"

ListaOrdenada::ListaOrdenada()
{
    l = (Lista *) malloc (sizeof(struct Lista));
    l->sig = NULL;
}


ListaOrdenada::~ListaOrdenada(){

    while(l->sig!=NULL)
    {
        Lista * aux = l->sig->sig;
        delete l->sig->elemento;
        free (l->sig);
        l->sig = aux;
    }

    free (l);
}

void ListaOrdenada::insertar(Tripleta * elemento){

    Lista * lista = l;

    if (lista->sig == NULL){
        Lista * nuevo = (Lista *) malloc (sizeof(struct Lista));
        nuevo->sig = NULL;
        nuevo->elemento = elemento;
        lista->sig = nuevo;
        return;
    }

    while (lista->sig != NULL){
        //if (lista->sig->elemento->index == elemento->index) return; //Quitar repetidos.
        if (lista->sig->elemento->index > elemento->index){
            Lista * nuevo = (Lista *) malloc (sizeof(struct Lista));
            nuevo->elemento = elemento;
            nuevo->sig = lista->sig;
            lista->sig = nuevo;
            return;
        }
        lista = lista->sig;
    }

    Lista * nuevo = (Lista *) malloc (sizeof(struct Lista));
    nuevo->elemento = elemento;
    nuevo->sig = NULL;
    lista->sig = nuevo;
}

Tripleta * ListaOrdenada::recuperar(){
    return l->sig->elemento;
}

void ListaOrdenada::borrarPrimero(){

    if (l->sig != NULL){
        Lista * aux = l->sig->sig;
        free (l->sig);
        l->sig = aux;
    }
}

bool ListaOrdenada::isVacia(){
    return l->sig == NULL;
}


