#include "GUI/windowsgeneral.h"
#include <QApplication>
#include <QTextCodec>
#include <cinterval.hpp>
#include <interval.hpp>
#include <iostream>

qint32 main(qint32 argc, char *argv[])
{

    /*interval <qreal> k (1, 500);

    complex <qreal> p (0.43, 1);

    cinterval <qreal> c  = complex <qreal> (0,1) +  interval<qreal>(0, 1);

    complex <qreal> p0 (0.006, -0.001);

    cinterval <qreal> f = ((k * c) / p) * p0;

    interval <qreal> b = abs(f);

    if (b.inf == 0){
        b.inf = 0.01;
    }

    interval<qreal> g = 20.0 * log10(b);
    interval<qreal> theta = arg(f) * 180.0 / M_PI;

    f = cinterval<qreal> (g, theta);

    cout << "c: [" +  QString::number(f.re.inf).toStdString() + ", " + QString::number(f.re.sup).toStdString() + "] + [" + QString::number(f.im.inf).toStdString() + ", " + QString::number(f.im.sup).toStdString() + "]i" << endl;

    return 0 ;*/


    QApplication a(argc, argv);

    WindowsGeneral * w = new WindowsGeneral();
    w->show();
    
    qint32 i = a.exec();

    delete w;
    return i;

}


