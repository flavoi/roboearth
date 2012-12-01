/****************************************************************************
** Meta object code from reading C++ file 'comthread.h'
**
** Created: Mon Nov 12 11:35:05 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/comthread.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'comthread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ComThread[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      11,   10,   10,   10, 0x05,
      46,   10,   10,   10, 0x05,

 // slots: signature, parameters, type, tag, flags
     111,   79,   10,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_ComThread[] = {
    "ComThread\0\0updateZaragozaDetectionImg(QImage)\0"
    "updateKinectDetectionImg(QImage)\0"
    "model_dir,model_type,model_name\0"
    "publishModelDir(QString,QString,QString)\0"
};

void ComThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ComThread *_t = static_cast<ComThread *>(_o);
        switch (_id) {
        case 0: _t->updateZaragozaDetectionImg((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 1: _t->updateKinectDetectionImg((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 2: _t->publishModelDir((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData ComThread::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ComThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_ComThread,
      qt_meta_data_ComThread, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ComThread::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ComThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ComThread::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ComThread))
        return static_cast<void*>(const_cast< ComThread*>(this));
    return QThread::qt_metacast(_clname);
}

int ComThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void ComThread::updateZaragozaDetectionImg(QImage _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ComThread::updateKinectDetectionImg(QImage _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
