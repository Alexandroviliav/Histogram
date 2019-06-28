#ifndef IMAGEACQUISITION_H
#define IMAGEACQUISITION_H
#define INTENSITY_SIZE 65536    // Размер массива Х
#define INTENSITY_MAX 65535     // Максимальный Y


#include <QObject>
#include "mytiff.h"
#include <QDebug>
#include <QVector>

class ImageAcquisition : public QObject
{
    Q_OBJECT
public:
    explicit ImageAcquisition(QObject *parent = nullptr);

    QVector<double> getVectorX();
    QVector<double> getVectorY();

    double getMinY();
    double getMaxY();

    void    setParameters(int &width, int &height, ushort *data);
    void    readImageFromFile();
    int     getWidth();
    int     getHeight();

private:
    myTIFF              *tif;
    QMap<int,int> *map;
//    const char          *imageName = "rec_1193.tif";
//    const char          *imageName = "s_0016.tif";
    const char          *imageName = "14-38-40.tif";

    uint    tifWidth;
    uint    tifHeight;
    double  minY = 0;
    double  maxY = 0;

    QVector<double> y;

    void readImageContinuously(int &width, int &height, ushort *data);
    void setMinMaxY(QVector<double>&);

signals:

public slots:
};

#endif // IMAGEACQUISITION_H
