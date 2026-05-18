#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "Tarantula.h"
#include "MotorController.h"
#include <QTableWidgetItem>
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

void MainWindow::sendCurrentBodyPose()
{
    static constexpr double SLIDER_XYZ_SCALE   = 0.001;
    static constexpr double SLIDER_ANGLE_SCALE = M_PI / 180.0;

    double dx    = ui->sliderBodyX->value() * SLIDER_XYZ_SCALE;
    double dy    = ui->sliderBodyY->value() * SLIDER_XYZ_SCALE;
    double dz    = ui->sliderBodyZ->value() * SLIDER_XYZ_SCALE;
    double roll  = ui->sliderBodyRoll->value() * SLIDER_ANGLE_SCALE;
    double pitch = ui->sliderBodyPitch->value() * SLIDER_ANGLE_SCALE;

    robot_->setBodyPose(dx, dy, dz, roll, pitch);
}

void MainWindow::on_btnResetPose_clicked()
{
    ui->sliderBodyX->blockSignals(true);
    ui->sliderBodyY->blockSignals(true);
    ui->sliderBodyZ->blockSignals(true);
    ui->sliderBodyRoll->blockSignals(true);
    ui->sliderBodyPitch->blockSignals(true);

    ui->sliderBodyX->setValue(0);
    ui->sliderBodyY->setValue(0);
    ui->sliderBodyZ->setValue(0);
    ui->sliderBodyRoll->setValue(0);
    ui->sliderBodyPitch->setValue(0);

    ui->sliderBodyX->blockSignals(false);
    ui->sliderBodyY->blockSignals(false);
    ui->sliderBodyZ->blockSignals(false);
    ui->sliderBodyRoll->blockSignals(false);
    ui->sliderBodyPitch->blockSignals(false);

    robot_->resetBodyPoseReference();
    robot_->setBodyPose(0.0, 0.0, 0.0, 0.0, 0.0);
}
