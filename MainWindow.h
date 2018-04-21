#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

enum handle_id_t {
	H_NONE = -1,
	H_TOPLEFT = 0,
	H_TOPRIGHT,
	H_BOTTOMLEFT,
	H_BOTTOMRIGHT,
};


class MainWindow : public QMainWindow
{
	Q_OBJECT
private:
	struct Private;
	Private *priv;


public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

private:
	Ui::MainWindow *ui;

	// QWidget interface
	handle_id_t HandleHitTest(QPoint pt) const;
protected:
	void paintEvent(QPaintEvent *event);

	// QWidget interface
protected:
	void mousePressEvent(QMouseEvent *event);

	// QWidget interface
protected:
	void mouseMoveEvent(QMouseEvent *event);

	// QWidget interface
protected:
	void mouseReleaseEvent(QMouseEvent *event);
};

#endif // MAINWINDOW_H
