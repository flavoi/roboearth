/****************************************************************************
** Meta object code from reading C++ file 'downloadobjectdialog.h'
**
** Created: Mon Nov 12 11:35:05 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/downloadobjectdialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'downloadobjectdialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_DownloadObjectDialog[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      22,   21,   21,   21, 0x08,
      48,   21,   21,   21, 0x08,
      74,   21,   21,   21, 0x08,
     111,   21,   21,   21, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_DownloadObjectDialog[] = {
    "DownloadObjectDialog\0\0on_cancelButton_clicked()\0"
    "on_searchButton_clicked()\0"
    "on_chooseDownloadDirButton_clicked()\0"
    "on_downloadButton_clicked()\0"
};

void DownloadObjectDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        DownloadObjectDialog *_t = static_cast<DownloadObjectDialog *>(_o);
        switch (_id) {
        case 0: _t->on_cancelButton_clicked(); break;
        case 1: _t->on_searchButton_clicked(); break;
        case 2: _t->on_chooseDownloadDirButton_clicked(); break;
        case 3: _t->on_downloadButton_clicked(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData DownloadObjectDialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject DownloadObjectDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_DownloadObjectDialog,
      qt_meta_data_DownloadObjectDialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &DownloadObjectDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *DownloadObjectDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *DownloadObjectDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_DownloadObjectDialog))
        return static_cast<void*>(const_cast< DownloadObjectDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int DownloadObjectDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
