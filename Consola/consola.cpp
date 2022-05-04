#include "consola.h"

consola::consola()
{

}

consola::~consola()
{

}


void consola::mostrar(){
    QProcess::execute("./qterminal/bin/qterminal -e ./muparserx/bin/example");
}
