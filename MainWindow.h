#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

// Declaración adelantada: la GUI solo conoce la interfaz de Tarantula
class Tarantula;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(Tarantula* robot, QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    // Actualización periódica de la UI (timer 100 ms)
    void updateUI();

    
    void on_btnStartAll_clicked();
    void on_btnStopAll_clicked();
    void on_btnStandUp_clicked();
    void on_btnSitDown_clicked();


    void on_btnSendCommandGrid_clicked();

    void on_btnEnableIndividual_clicked();
    void on_btnSendIndividual_clicked();

    
    void on_sliderBodyX_valueChanged(int value);
    void on_sliderBodyY_valueChanged(int value);
    void on_sliderBodyZ_valueChanged(int value);
    void on_sliderBodyRoll_valueChanged(int value);
    void on_sliderBodyPitch_valueChanged(int value);
    void on_btnResetPose_clicked();

private:
    Ui::MainWindow* ui;
    Tarantula*      robot_;
    QTimer*         timer_;

    
    void initTable();
    void updateStatusLeds();
    void updateMotorTable();
    void updateSpiderWidget();
    QString buildStiffnessLabel(bool online, bool active, float kp) const;
    void    setTableCell(int row, int col, const QString& text, const QString& color);

    
    void sendCurrentBodyPose();
};

#endif // MAINWINDOW_H
