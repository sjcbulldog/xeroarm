#pragma once

#include "CentralWidget.h"
#include "ArmDataModel.h"
#include "PathsDisplayWidget.h"
#include "WaypointWindow.h"
#include "PlotWindow.h"
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>


class xeroarm : public QMainWindow
{
    Q_OBJECT

public:
    xeroarm(QWidget *parent = nullptr);
    ~xeroarm();

protected:
    void closeEvent(QCloseEvent* ev) override;

private:
    void createWindows();
    void createMenus();

    void saveFile();
    void saveAsFile();
    void closeFile();
    void openFile();

    void resetView();

    void save(const QString& path);
    
    void progress(const QString& msg);
    void timeChange(double t);
    void mouseMove(const Translation2d& pos);

    void setIKJacobian();

private:
    static constexpr const char* GeometrySetting = "geometry";
    static constexpr const char* WindowStateSetting = "windowState";
    static constexpr const char* MainSplitterSettings = "mainSplitter";
    static constexpr const char* PlotSplitterSettings = "plotSplitter";

private:
    QSettings settings_;
    ArmDataModel model_;
    CentralWidget* central_;

    QDockWidget* dock_plot_win_;
    PlotWindow* plot_win_;

    QDockWidget* path_display_dock_;
    PathsDisplayWidget* path_display_;

    QDockWidget* waypoint_display_dock_;
    WaypointWindow* waypoint_display_;

    QLabel* time_text_;
    QLabel* pos_text_;
    QLabel* status_text_;
    QLabel* ik_type_text_;

    QString filename_;

    QMenu* file_menu_;
    QMenu* window_menu_;
    QMenu* ik_type_;
    QActionGroup* ik_type_group_;
    QAction* ik_jacobian_;
    QAction* ik_annealing_;
};
