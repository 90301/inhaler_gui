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
 //draws the text to the screen.
 void draw(QPainter& p) {
   //QColor qcolor(122,122,122);
   //p.setPen(Qt::black);
   QFont font = p.font();   
   font.setPointSize(textSize);
  p.setFont(font);
   QString qs = QString::fromStdString(text);  
   
  p.drawText((int)x,(int)y,qs);
  
  //std::cout << "drawing text: " << text << " | " << qs.toStdString() << std::endl;
 }
};

/*
 * PERSISTENT GUI
 * persistent gui
 * all elements that say use a string "ID" to identify them.
 */

class PLine : public I_Shape {
public:
 double x1,y1,x2,y2;
 std::string id;
 PLine(double x1,double y1,double x2,double y2,std::string& id) {
  this->x1 = x1;
  this->x2 = x2;
  this->y1 = y1;
  this->y2 = y2;
  this->id = id;
 }
 PLine() {
   //This should hopefully never run.
   std::cout << "Persistent line (PLine) created with blank constructor." << std::endl;
 }
 
 //update method
 void updateP(PLine& pline) {
   this->x1 = pline.x1;
   this->x2 = pline.x2;
   this->y1 = pline.y1;
   this->y2 = pline.y2;
 }
 //over ridden method
 void draw(QPainter& p) {
  p.drawLine (this->x1,this->y1,this->x2,this->y2);
 }
};

class PText : public I_Shape {
public:
 double x,y;
 int textSize;
 std::string text;
 std::string id;
 PText(double& x,double& y,int& textSize, std::string& text, std::string& id) {
   this->x = x;
   this->y = y;
   this->textSize = textSize;
   this->text = text;
   this->id = id;
 }
 PText() {
      //This should hopefully never run.
   std::cout << "Persistent text (Ptext) created with blank constructor." << std::endl;
 }
 //update methods
 void updateP(std::string& text) {
   this->text = text;
 }
 void updateP(double& x,double& y,int& textSize, std::string& text) {
   this->x = x;
   this->y = y;
   this->textSize = textSize;
   this->text = text;
 }
 void updateP(PText& ptext) {
   this->x = ptext.x;
   this->y = ptext.y;
   this->textSize = ptext.textSize;
   this->text = ptext.text;
 }
 //over ridden method
 //draws the text to the screen.
 void draw(QPainter& p) {
   QFont font = p.font();   
   font.setPointSize(textSize);
  p.setFont(font);
   QString qs = QString::fromStdString(text);  
   
  p.drawText((int)x,(int)y,qs);
  
  //std::cout << "drawing text: " << text << " | " << qs.toStdString() << std::endl;
 }
};

/*
 * Holds all persistent data structures
 */
class PWorkspace {
public:
  std::map<std::string,PLine> pLines;
  std::map<std::string,PText> pTexts;
  void clear() {
    pLines.clear();
    pTexts.clear();
  }
  void addPLine(PLine& pLine) {
    std::map<std::string,PLine>::iterator it = pLines.find(pLine.id);
    
    if (it != pLines.end()) {
      //update
      pLines[pLine.id] = pLine;
    } else {
      //create a new line for that id
    pLines[pLine.id] = pLine;
    }
  }
  
  void drawEverything(QPainter& p) {
    std::map<std::string,PLine>::iterator pLineIt;
    std::map<std::string,PText>::iterator pTextIt;
    for (pLineIt = pLines.begin(); pLineIt!=pLines.end();pLineIt++) {
      (*pLineIt).second.draw(p);
    }
     for (pTextIt = pTexts.begin(); pTextIt!=pTexts.end();pTextIt++) {
      (*pTextIt).second.draw(p);
    }
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
void clearGUI(const inhaler_gui::draw_textConstPtr& msg) {
  lines.clear();
  texts.clear();
  
  /*
  double x = msg->x;
  double y = msg->y;
  int textSize = msg->textSize;
  std::string text = msg->text;
  Text t(x,y,textSize,text);
  texts.push_back (t);
   */
  std::cout << "added text!" << std::endl;
}

const static double VERSION_NUMBER = 1.45;
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
    ros::Subscriber clearSub = n.subscribe ("inhalerGUI_Clear",1000,clearGUI);
    
    
    widget.show();
    
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    //ros::waitForShutdown();
    app.exec();
    //app.exec();
    
    return 0;
}
