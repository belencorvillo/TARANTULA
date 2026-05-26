#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "Tarantula.h"
#include "MotorController.h"
#include <QTableWidgetItem>
#include <QGroupBox>
#include <QGridLayout>
#include <QString>
#include <thread>

static constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979f;
static constexpr float DEG_TO_RAD = 3.14159265358979f / 180.0f;

// Escala de los sliders: 1 unidad = 1 mm para XYZ, 1° para ángulos
//static constexpr double SLIDER_XYZ_SCALE   = 0.001;   // slider int → metros
//static constexpr double SLIDER_ANGLE_SCALE = M_PI / 180.0; // slider int → rad



MainWindow::MainWindow(Tarantula* robot, QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , robot_(robot)
{
    ui->setupUi(this);
    initTable();

    // Recortar la anchura horizontal de los botones principales del panel derecho
    ui->btnStartAll->setMaximumWidth(180);
    ui->btnStopAll->setMaximumWidth(180);
    ui->btnStandUp->setMaximumWidth(180);
    ui->btnSitDown->setMaximumWidth(180);

    ui->btnStartAll->setFocusPolicy(Qt::NoFocus);
    ui->btnStopAll->setFocusPolicy(Qt::NoFocus);
    ui->btnStandUp->setFocusPolicy(Qt::NoFocus);
    ui->btnSitDown->setFocusPolicy(Qt::NoFocus);

    // Crear el GroupBox para el D-Pad del Trote
    QGroupBox* trotGroup = new QGroupBox("Trote Alterno", this);
    trotGroup->setStyleSheet(
        "QGroupBox { color: #f9e2af; font-weight: bold; border: 2px solid #313244; border-radius: 6px; padding: 10px; }"
    );
    trotGroup->setMaximumWidth(220);

    QGridLayout* gridLayout = new QGridLayout(trotGroup);
    gridLayout->setSpacing(6);
    gridLayout->setContentsMargins(5, 5, 5, 5);

    btnTrotUp_ = new QPushButton("▲", this);
    btnTrotDown_ = new QPushButton("▼", this);
    btnTrotLeft_ = new QPushButton("◀", this);
    btnTrotRight_ = new QPushButton("▶", this);

    btnTrotUp_->setFocusPolicy(Qt::NoFocus);
    btnTrotDown_->setFocusPolicy(Qt::NoFocus);
    btnTrotLeft_->setFocusPolicy(Qt::NoFocus);
    btnTrotRight_->setFocusPolicy(Qt::NoFocus);

    // Estilo elegante adaptado al tema oscuro
    QString btnStyle = "QPushButton { background-color: #313244; color: #cdd6f4; border: 2px solid #45475a; border-radius: 6px; min-width: 45px; min-height: 45px; max-width: 45px; max-height: 45px; font-weight: bold; font-size: 16px; }"
                       "QPushButton:pressed { background-color: #f9e2af; color: #1e1e2e; }"
                       "QPushButton:checked { background-color: #f9e2af; color: #1e1e2e; }";
    btnTrotUp_->setStyleSheet(btnStyle);
    btnTrotDown_->setStyleSheet(btnStyle);
    btnTrotLeft_->setStyleSheet(btnStyle);
    btnTrotRight_->setStyleSheet(btnStyle);

    gridLayout->addWidget(btnTrotUp_, 0, 1);
    gridLayout->addWidget(btnTrotLeft_, 1, 0);
    gridLayout->addWidget(btnTrotRight_, 1, 2);
    gridLayout->addWidget(btnTrotDown_, 2, 1);

    // Conectar señales del D-Pad físico
    connect(btnTrotUp_, &QPushButton::pressed, [this]() { keyUpPressed_ = true; updateTrotVelocity(); });
    connect(btnTrotUp_, &QPushButton::released, [this]() { keyUpPressed_ = false; updateTrotVelocity(); });

    connect(btnTrotDown_, &QPushButton::pressed, [this]() { keyDownPressed_ = true; updateTrotVelocity(); });
    connect(btnTrotDown_, &QPushButton::released, [this]() { keyDownPressed_ = false; updateTrotVelocity(); });

    connect(btnTrotLeft_, &QPushButton::pressed, [this]() { keyLeftPressed_ = true; updateTrotVelocity(); });
    connect(btnTrotLeft_, &QPushButton::released, [this]() { keyLeftPressed_ = false; updateTrotVelocity(); });

    connect(btnTrotRight_, &QPushButton::pressed, [this]() { keyRightPressed_ = true; updateTrotVelocity(); });
    connect(btnTrotRight_, &QPushButton::released, [this]() { keyRightPressed_ = false; updateTrotVelocity(); });

    // Extraer los botones de acción del panel derecho y reagruparlos horizontalmente junto con el D-Pad
    ui->verticalLayout_Right->removeWidget(ui->btnStartAll);
    ui->verticalLayout_Right->removeWidget(ui->btnStopAll);
    ui->verticalLayout_Right->removeWidget(ui->btnStandUp);
    ui->verticalLayout_Right->removeWidget(ui->btnSitDown);

    QVBoxLayout* actionButtonsLayout = new QVBoxLayout();
    actionButtonsLayout->setSpacing(6);
    actionButtonsLayout->setContentsMargins(0, 0, 0, 0);
    actionButtonsLayout->addWidget(ui->btnStartAll);
    actionButtonsLayout->addWidget(ui->btnStopAll);
    actionButtonsLayout->addWidget(ui->btnStandUp);
    actionButtonsLayout->addWidget(ui->btnSitDown);

    QHBoxLayout* topHorizontalLayout = new QHBoxLayout();
    topHorizontalLayout->setSpacing(15);
    topHorizontalLayout->setContentsMargins(0, 0, 0, 0);
    topHorizontalLayout->addLayout(actionButtonsLayout);
    topHorizontalLayout->addWidget(trotGroup);

    // Añadir un spacer horizontal a la derecha para que todo se agrupe de forma elegante a la izquierda
    topHorizontalLayout->addSpacerItem(new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Minimum));

    // Insertar este layout combinado horizontal justo debajo de las etiquetas de USB/CAN (índice 1 en verticalLayout_Right)
    ui->verticalLayout_Right->insertLayout(1, topHorizontalLayout);

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow::updateUI);
    timer_->start(100);  // 10 Hz de refresco de la UI
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::initTable()
{
    ui->tableMotors->setColumnCount(3);
    ui->tableMotors->setHorizontalHeaderLabels({"Motor", "Posición", "Rigidez"});
    ui->tableMotors->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}



void MainWindow::updateUI()
{
    updateStatusLeds();
    updateMotorTable();
    updateSpiderWidget();
}

void MainWindow::updateStatusLeds()
{
    bool usb_ok = g_comm && g_comm->connected();
    QString green = "background-color: #a6e3a1; color: #11111b; border-radius: 4px;";
    QString red   = "background-color: #f38ba8; color: #11111b; border-radius: 4px;";

    ui->lblUsbStatus->setStyleSheet(usb_ok ? green : red);
    ui->lblCanStatus->setStyleSheet(robot_->isAnyMotorOnline() ? green : red);
}

void MainWindow::updateMotorTable()
{
    static const std::vector<uint32_t> MOTOR_IDS = {
        11,12,13, 21,22,23, 31,32,33, 41,42,43
    };

    ui->tableMotors->setRowCount(static_cast<int>(MOTOR_IDS.size()));
    int row = 0;

    for (uint32_t id : MOTOR_IDS) {
        MotorController* ctrl = g_controllers[id];
        if (!ctrl) { ++row; continue; }

        bool  online   = ctrl->motor_online.load(std::memory_order_relaxed);
        bool  active   = ctrl->active.load(std::memory_order_relaxed);
        float pos_deg  = ctrl->last_known_pos.load(std::memory_order_relaxed) * RAD_TO_DEG;
        float kp       = ctrl->target_kp.load(std::memory_order_relaxed);

        QString stiffness_label = buildStiffnessLabel(online, active, kp);
        QString color = online ? "#cdd6f4" : "#f38ba8";

        setTableCell(row, 0, QString::number(id),                      color);
        setTableCell(row, 1, online ? QString::number(pos_deg,'f',1)+"°" : "-", color);
        setTableCell(row, 2, stiffness_label, active ? "#a6e3a1" : "#f38ba8");
        ++row;
    }
}

QString MainWindow::buildStiffnessLabel(bool online, bool active, float kp) const
{
    if (!online) return "DESCONECTADO";
    if (!active) return "OFF";
    if (kp <= 0.0f)  return "ON (Suave)";
    if (kp <= 5.0f)  return "Perfil 1";
    if (kp <= 10.0f) return "Perfil 2";
    if (kp <= 20.0f) return "Perfil 3";
    if (kp <= 35.0f) return "Perfil 4";
    return "Perfil 5";
}

void MainWindow::setTableCell(int row, int col, const QString& text, const QString& color)
{
    QTableWidgetItem* item = ui->tableMotors->item(row, col);
    if (!item) {
        item = new QTableWidgetItem();
        item->setTextAlignment(Qt::AlignCenter);
        ui->tableMotors->setItem(row, col, item);
    }
    item->setText(text);
    item->setForeground(QBrush(QColor(color)));
}

void MainWindow::updateSpiderWidget()
{
    static const std::vector<uint32_t> MOTOR_IDS = {
        11,12,13, 21,22,23, 31,32,33, 41,42,43
    };
    for (uint32_t id : MOTOR_IDS) {
        MotorController* ctrl = g_controllers[id];
        if (!ctrl) continue;
        bool  online  = ctrl->motor_online.load(std::memory_order_relaxed);
        float pos_rad = ctrl->last_known_pos.load(std::memory_order_relaxed);
        ui->spiderWidget->updateMotor(id, pos_rad, online);
    }
}


void MainWindow::on_btnStartAll_clicked()
{
    robot_->enableAllLegs();
}

void MainWindow::on_btnStopAll_clicked()
{
    robot_->disableAllLegs();
}

void MainWindow::on_btnStandUp_clicked()
{
    robot_->standUp();
}

void MainWindow::on_btnSitDown_clicked()
{
    robot_->sitDown();
}


void MainWindow::on_btnSendCommandGrid_clicked()
{
    float p1 = ui->txtPos1->text().toFloat();
    float p2 = ui->txtPos2->text().toFloat();
    float p3 = ui->txtPos3->text().toFloat();
    int   r1 = ui->txtRig1->text().toInt();
    int   r2 = ui->txtRig2->text().toInt();
    int   r3 = ui->txtRig3->text().toInt();

    std::thread([this, p1, p2, p3, r1, r2, r3]() {
        for (int leg = 1; leg <= 4; ++leg) {
            robot_->moveLegJoint(leg, 1, p1, r1);
            robot_->moveLegJoint(leg, 2, p2, r2);
            robot_->moveLegJoint(leg, 3, p3, r3);
        }
    }).detach();
}

void MainWindow::on_btnEnableIndividual_clicked()
{
    bool ok;
    int id = ui->txtMoverID->text().toInt(&ok);
    if (!ok) return;

    int leg_id = id / 10;
    int joint = id % 10;
    if (leg_id >= 1 && leg_id <= 4) {
        robot_->getLeg(leg_id).enableJoint(joint);
    }
}

void MainWindow::on_btnSendIndividual_clicked()
{
    bool ok_id, ok_pos, ok_rig;
    int   id  = ui->txtMoverID->text().toInt(&ok_id);
    float pos = ui->txtMoverPos->text().toFloat(&ok_pos);
    int   rig = ui->txtMoverRigidez->text().toInt(&ok_rig);
    if (!ok_id || !ok_pos || !ok_rig) return;

    int leg_id = id / 10;
    int joint = id % 10;
    if (leg_id >= 1 && leg_id <= 4) {
        std::thread([this, leg_id, joint, pos, rig]() {
            robot_->getLeg(leg_id).moveJoint(joint, pos, rig);
        }).detach();
    }
}


void MainWindow::on_sliderBodyX_valueChanged(int)     { sendCurrentBodyPose(); }
void MainWindow::on_sliderBodyY_valueChanged(int)     { sendCurrentBodyPose(); }
void MainWindow::on_sliderBodyZ_valueChanged(int)     { sendCurrentBodyPose(); }
void MainWindow::on_sliderBodyRoll_valueChanged(int)  { sendCurrentBodyPose(); }
void MainWindow::on_sliderBodyPitch_valueChanged(int) { sendCurrentBodyPose(); }
void MainWindow::on_sliderBodyYaw_valueChanged(int)   { sendCurrentBodyPose(); }

void MainWindow::sendCurrentBodyPose()
{
    static constexpr double SLIDER_XYZ_SCALE   = 0.001; // 1 unit = 1 mm (0.001 m)
    static constexpr double SLIDER_ANGLE_SCALE = M_PI / 180.0;

    int valX = ui->sliderBodyX->value();
    int valY = ui->sliderBodyY->value();
    int valZ = ui->sliderBodyZ->value();
    int valRoll = ui->sliderBodyRoll->value();
    int valPitch = ui->sliderBodyPitch->value();
    int valYaw = ui->sliderBodyYaw->value();

    ui->lblBodyX->setText(QString("%1 mm").arg(valX));
    ui->lblBodyY->setText(QString("%1 mm").arg(valY));
    ui->lblBodyZ->setText(QString("%1 mm").arg(valZ));
    ui->lblBodyRoll->setText(QString("%1°").arg(valRoll));
    ui->lblBodyPitch->setText(QString("%1°").arg(valPitch));
    ui->lblBodyYaw->setText(QString("%1°").arg(valYaw));

    double dx    = valX * SLIDER_XYZ_SCALE;
    double dy    = valY * SLIDER_XYZ_SCALE;
    double dz    = valZ * SLIDER_XYZ_SCALE;
    double roll  = valRoll * SLIDER_ANGLE_SCALE;
    double pitch = valPitch * SLIDER_ANGLE_SCALE;
    double yaw   = valYaw * SLIDER_ANGLE_SCALE;

    robot_->setBodyPose(dx, dy, dz, roll, pitch, yaw);
}

void MainWindow::on_btnResetPose_clicked()
{
    ui->sliderBodyX->blockSignals(true);
    ui->sliderBodyY->blockSignals(true);
    ui->sliderBodyZ->blockSignals(true);
    ui->sliderBodyRoll->blockSignals(true);
    ui->sliderBodyPitch->blockSignals(true);
    ui->sliderBodyYaw->blockSignals(true);

    ui->sliderBodyX->setValue(0);
    ui->sliderBodyY->setValue(0);
    ui->sliderBodyZ->setValue(0);
    ui->sliderBodyRoll->setValue(0);
    ui->sliderBodyPitch->setValue(0);
    ui->sliderBodyYaw->setValue(0);

    ui->sliderBodyX->blockSignals(false);
    ui->sliderBodyY->blockSignals(false);
    ui->sliderBodyZ->blockSignals(false);
    ui->sliderBodyRoll->blockSignals(false);
    ui->sliderBodyPitch->blockSignals(false);
    ui->sliderBodyYaw->blockSignals(false);

    ui->lblBodyX->setText("0 mm");
    ui->lblBodyY->setText("0 mm");
    ui->lblBodyZ->setText("0 mm");
    ui->lblBodyRoll->setText("0°");
    ui->lblBodyPitch->setText("0°");
    ui->lblBodyYaw->setText("0°");

    robot_->standUp();
}

void MainWindow::updateTrotVelocity()
{
    float vx = 0.0f;
    float vy = 0.0f;

    if (keyUpPressed_)    vx += 0.04f; // Adelante +X
    if (keyDownPressed_)  vx -= 0.04f; // Atrás -X
    if (keyLeftPressed_)  vy -= 0.04f; // Izquierda -Y (Y es positivo a la derecha)
    if (keyRightPressed_) vy += 0.04f; // Derecha +Y

    bool walking = (std::abs(vx) > 0.001f || std::abs(vy) > 0.001f);

    if (walking) {
        if (!robot_->isGaitActive()) {
            robot_->startGait();
        }
        robot_->setGaitVelocity(vx, vy);
    } else {
        if (robot_->isGaitActive()) {
            robot_->stopGait();
        }
    }
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    if (event->isAutoRepeat()) return;

    bool handled = false;
    switch (event->key()) {
    case Qt::Key_Up:
        keyUpPressed_ = true;
        if (btnTrotUp_) btnTrotUp_->setDown(true);
        handled = true;
        break;
    case Qt::Key_Down:
        keyDownPressed_ = true;
        if (btnTrotDown_) btnTrotDown_->setDown(true);
        handled = true;
        break;
    case Qt::Key_Left:
        keyLeftPressed_ = true;
        if (btnTrotLeft_) btnTrotLeft_->setDown(true);
        handled = true;
        break;
    case Qt::Key_Right:
        keyRightPressed_ = true;
        if (btnTrotRight_) btnTrotRight_->setDown(true);
        handled = true;
        break;
    default:
        break;
    }

    if (handled) {
        updateTrotVelocity();
    } else {
        QMainWindow::keyPressEvent(event);
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
    if (event->isAutoRepeat()) return;

    bool handled = false;
    switch (event->key()) {
    case Qt::Key_Up:
        keyUpPressed_ = false;
        if (btnTrotUp_) btnTrotUp_->setDown(false);
        handled = true;
        break;
    case Qt::Key_Down:
        keyDownPressed_ = false;
        if (btnTrotDown_) btnTrotDown_->setDown(false);
        handled = true;
        break;
    case Qt::Key_Left:
        keyLeftPressed_ = false;
        if (btnTrotLeft_) btnTrotLeft_->setDown(false);
        handled = true;
        break;
    case Qt::Key_Right:
        keyRightPressed_ = false;
        if (btnTrotRight_) btnTrotRight_->setDown(false);
        handled = true;
        break;
    default:
        break;
    }

    if (handled) {
        updateTrotVelocity();
    } else {
        QMainWindow::keyReleaseEvent(event);
    }
}
