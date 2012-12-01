/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Mon Nov 12 11:33:37 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
      39,   35,   11,   11, 0x0a,
      65,   11,   11,   11, 0x0a,
      86,   11,   11,   11, 0x08,
     139,   11,   11,   11, 0x08,
     181,   11,   11,   11, 0x08,
     205,   11,   11,   11, 0x08,
     232,  227,   11,   11, 0x08,
     272,  227,   11,   11, 0x08,
     311,   11,   11,   11, 0x08,
     354,  346,   11,   11, 0x08,
     387,   11,   11,   11, 0x08,
     411,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0changedBoxSize(double)\0"
    "msg\0addStatusMessage(QString)\0"
    "insertCloud(QString)\0"
    "on_boundingBoxSizeDoubleSpinBox_valueChanged(double)\0"
    "on_cloudListWidget_itemSelectionChanged()\0"
    "on_saveButton_clicked()\0on_reButton_clicked()\0"
    "arg1\0on_objectClassEdit_textChanged(QString)\0"
    "on_objectNameEdit_textChanged(QString)\0"
    "on_apiKeyEdit_textChanged(QString)\0"
    "checked\0on_startstopbutton_toggled(bool)\0"
    "on_loadButton_clicked()\0"
    "on_actionDelete_triggered()\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->changedBoxSize((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 1: _t->addStatusMessage((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->insertCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->on_boundingBoxSizeDoubleSpinBox_valueChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 4: _t->on_cloudListWidget_itemSelectionChanged(); break;
        case 5: _t->on_saveButton_clicked(); break;
        case 6: _t->on_reButton_clicked(); break;
        case 7: _t->on_objectClassEdit_textChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 8: _t->on_objectNameEdit_textChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 9: _t->on_apiKeyEdit_textChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 10: _t->on_startstopbutton_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 11: _t->on_loadButton_clicked(); break;
        case 12: _t->on_actionDelete_triggered(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QWidget::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::changedBoxSize(double _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
