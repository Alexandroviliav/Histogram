#ifndef MYTIFF_H
#define MYTIFF_H

#include <QObject>
#include "tiffio2.h"
#include <QDebug>
#include <QLibrary>

class myTIFF : public QObject
{
    Q_OBJECT
public:
//    explicit myTIFF(QObject *parent = nullptr);
    myTIFF();
    ushort * openFile(const char *  path, uint * width, uint * height);
        void writeFile(const char * path, ushort *data, uint  width, uint  height);

        ushort * data;

        typedef TIFF* (*TIFFOpen)(const char*, const char*);
        TIFFOpen myTIFFOpen;

        typedef int (*TIFFGetField)(TIFF*, ttag_t, ...);
        TIFFGetField myTIFFGetField;

        typedef int (*TIFFReadScanline)(TIFF*, tdata_t, uint32, tsample_t);
        TIFFReadScanline myTIFFReadScanline;

        typedef tdata_t (*_TIFFmalloc)(tsize_t);
        _TIFFmalloc my_TIFFmalloc;

        typedef tsize_t (*TIFFScanlineSize)(TIFF*);
        TIFFScanlineSize myTIFFScanlineSize;

        typedef int (*TIFFWriteScanline)(TIFF*, tdata_t, uint32, tsample_t);
        TIFFWriteScanline myTIFFWriteScanline;

        typedef uint32 (*TIFFDefaultStripSize)(TIFF*, uint32);
        TIFFDefaultStripSize myTIFFDefaultStripSize;

        typedef int (*TIFFSetField)(TIFF*, ttag_t, ...);
        TIFFSetField myTIFFSetField;

        typedef void (*TIFFClose)(TIFF*);
        TIFFClose myTIFFClose;

        typedef void (*TIFFSetupStrips)(TIFF*);
        TIFFSetupStrips myTIFFSetupStrips;

        uint getWidth(const char *path);
        uint getHeight(const char *path);


signals:

public slots:
};

#endif // MYTIFF_H
