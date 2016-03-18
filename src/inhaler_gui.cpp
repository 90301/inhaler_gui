#include <QApplication>
#include <QtWidgets>
#include <vector>
#include <qt5/QtGui/qpainter.h>
#include <iostream>

#include "ros/ros.h"

#include "inhaler_gui/draw_text.h"
#include "inhaler_gui/draw_line.h"


/**
 * Base class for all other shapes
 */
class I_Shape {
public:
 int x,y;
 void draw(QPainter& p){
  p.drawLine (0,0,100,100);
 };
};
class Line : public I_Shape {
public:
 double x1,y1,x2,y2;
 Line(double x1,double y1,double x2,double y2) {
  this->x1 = x1;
  this->x2 = x2;
  this->y1 = y1;
  this->y2 = y2;
 }
 //over ridden method
 void draw(QPainter& p) {
  p.drawLine (this->x1,this->y1,this->x2,this->y2);
 }
};
class Text : public I_Shape {
public:
 double x,y;
 int textSize;
 std::string text;
 Text(double& x,double& y,int& textSize, std::string& text) {
   this->x = x;
   this->y = y;
   this->textSize = textSize;
   this->text = text;
 }
 //over ridden method
 void draw(QPainter& p) {
   QString qs;
   qs.fromStdString(text);
  p.drawText((int)x,(int)y,qs);
 }
};

class MyWidget : public QWidget
{
public:
    MyWidget();
    //std::vector<I_Shape> shapes;
protected:
    void paintEvent(QPaintEvent *);
    
};

std::vector<Line> lines;
std::vector<Text> texts;


MyWidget::MyWidget()
{
    QPalette palette(MyWidget::palette());
    palette.setColor(backgroundRole(), Qt::white);
    setPalette(palette);
}

void MyWidget::paintEvent(QPaintEvent *)
{
 
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(Qt::darkGreen);
    //painter.drawRect(1, 1, 400, 400);//x1 y1 x2 y2

    painter.setPen(Qt::darkGray);
    //painter.drawLine(2, 2, 400, 400);
    
    for (int i=0;i<lines.size ();i++) {
      lines[i].draw(painter);
     }
    for (int i=0;i<texts.size ();i++) {
      texts[i].draw(painter);
     }
    
    
}

/*
 * CALL BACK FUNCTIONS
 * 
 * CCCCCCC     AAAAA    L         L         BBBBBB     AAAAA    CCCCCC  K    k
 * C          A     A   L         L         B     B   A     A   C       K  k
 * C          AAAAAAA   L         L         BBBBBBB   AAAAAAA   C       KK
 * C          A     A   L         L         B     B   A     A   C       K  k
 * CCCCCCC    A     A   LLLLLLL   LLLLLLL   BBBBBB    A     A   CCCCCC  K    k
 */
//void addLine(const inhaler_gui::draw_line msg) {
 
//}
void addLine(const inhaler_gui::draw_lineConstPtr& msg) {
  double x1 = msg->x1;
  double y1 = msg->y1;
  double x2 = msg->x2;
  double y2 = msg->y2;
  Line l(x1,y1,x2,y2);
  lines.push_back (l);
  std::cout << "added line: " << x1 << " , " << y1 << " -> " << x2 << " , " << y2 << std::endl;
}
void addText(const inhaler_gui::draw_textConstPtr& msg) {
  double x = msg->x;
  double y = msg->y;
  int textSize = msg->textSize;
  std::string text = msg->text;
  Text t(x,y,textSize,text);
  texts.push_back (t);
  std::cout << "added text!" << std::endl;
}

const static double VERSION_NUMBER = 1.3;
int main(int argc, char *argv[])
{
  
  std::cout << "Inhaler GUI Version: " << VERSION_NUMBER << " started." << std::endl;
 ros::init (argc, argv, "inhaler_gui_server");
    QApplication app(argc, argv);
    MyWidget widget;
    
    Line l(100,100,210,200);
    lines.push_back (l);
    //ROS subscribing
    ros::NodeHandle n;
    
    ros::Subscriber lineSub = n.subscribe ("inhalerGUI_Line",1000,addLine);
    ros::Subscriber textSub = n.subscribe ("inhalerGUI_Text",1000,addText);
    
    
    
    widget.show();
    
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    //ros::waitForShutdown();
    app.exec();
    //app.exec();
    
    return 0;
}
