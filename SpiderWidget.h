#ifndef SPIDERWIDGET_H
#define SPIDERWIDGET_H

#include <QWidget>
#include <map>

struct JointData {
    float pos = 0.0f; // en radianes
    bool online = false;
};

class SpiderWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SpiderWidget(QWidget *parent = nullptr);

    // Actualiza la posición de un motor específico
    void updateMotor(uint8_t id, float pos_rad, bool online);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    std::map<uint8_t, JointData> motors;
    
    // Función auxiliar para dibujar una pata
    void drawLeg(QPainter &painter, float centerX, float centerY, float baseAngleDeg, uint8_t baseId, float bodyRadius, float tiltFactor);
};

#endif // SPIDERWIDGET_H
