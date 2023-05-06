#pragma once

#include "CentralWidget.h"
#include "ArmDataModel.h"
#include "PathsDisplayWidget.h"
#include "WaypointWindow.h"
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
    void addJoint();
    void createPath();

    void save(const QString& path);
    

private:
    static constexpr const char* GeometrySetting = "geometry";
    static constexpr const char* WindowStateSetting = "windowState";
    static constexpr const char* MainSplitterSettings = "mainSplitter";
    static constexpr const char* BottomSplitterSettings = "bottomSplitter";

private:
    QSettings settings_;
    ArmDataModel model_;
    CentralWidget* central_;

    QDockWidget* path_display_dock_;
    PathsDisplayWidget* path_display_;

    QDockWidget* waypoint_display_dock_;
    WaypointWindow* waypoint_display_;

    QString filename_;

    QMenu* file_menu_;
    QMenu* view_menu_;
};
