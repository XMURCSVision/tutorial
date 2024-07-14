#include <GL/glut.h>
#include <math.h>

#include <iostream>
#define COLOR 8.0
#define PIX ((GLfloat)(1.0 / COLOR))
#define STEP 1.0f
using namespace std;

// 旋转初始的角度
GLfloat angle = 0.0f;
// 设置旋转轴：两个三维的点确定的旋转轴
GLfloat axis[][3] = {0.0f, 0.5f, 0.5f, 1.0f, 0.5f, 0.5f};

// 绘制一个色彩像素点（实际是小正方形）
// 返回值二维数组pix[4][3]，分别存储正方形四个顶点坐标，同时第一个顶点坐标就是颜色值
// 参数direction表示像素点所在平面是正面还是背面，0表示背面，1表示正面
// 参数main_color表示像素点所在平面的基调颜色，0表示红色，1表示绿色，2表示蓝色
//		同时main_color也决定了像素点所在的平面法向，红色平面与x轴垂直，绿色y轴，蓝色z轴
// 参数relative_x和relative_y表示像素点在立方体平面上的相对位置
//		同时相对位置也和基调颜色共同决定了像素点的颜色RGB值
GLfloat** get_pix(unsigned int front_back, unsigned int direction,
                  GLfloat relative_x, GLfloat relative_y) {
  // 申请存储像素正方形四个顶点的二维数组pix[4][3]
  GLfloat** pix = (GLfloat**)malloc(sizeof(GLfloat*) * 4);
  if (pix == NULL) return NULL;
  for (unsigned int i = 0; i < 4; i++) {
    pix[i] = (GLfloat*)malloc(sizeof(GLfloat) * 3);
    if (pix[i] == NULL) return NULL;
  }

  if (front_back == 0) {
    // 像素点位于朝向背面的三个面，正方形的顶点顺时针顺序排列
    if (direction == 0) {
      // 像素点位于YOZ平面（右后侧面）
      pix[0][0] = 0.0f;
      pix[0][1] = relative_x;
      pix[0][2] = relative_y;
      pix[1][0] = 0.0f;
      pix[1][1] = relative_x;
      pix[1][2] = relative_y + PIX;
      pix[2][0] = 0.0f;
      pix[2][1] = relative_x + PIX;
      pix[2][2] = relative_y + PIX;
      pix[3][0] = 0.0f;
      pix[3][1] = relative_x + PIX;
      pix[3][2] = relative_y;
    } else if (direction == 1) {
      // 像素点位于ZOX平面（左后侧面）
      pix[0][0] = relative_y;
      pix[0][1] = 0.0f;
      pix[0][2] = relative_x;
      pix[1][0] = relative_y + PIX;
      pix[1][1] = 0.0f;
      pix[1][2] = relative_x;
      pix[2][0] = relative_y + PIX;
      pix[2][1] = 0.0f;
      pix[2][2] = relative_x + PIX;
      pix[3][0] = relative_y;
      pix[3][1] = 0.0f;
      pix[3][2] = relative_x + PIX;
    } else if (direction == 2) {
      // 像素点位于XOY平面（底面）
      pix[0][0] = relative_x;
      pix[0][1] = relative_y;
      pix[0][2] = 0.0f;
      pix[1][0] = relative_x;
      pix[1][1] = relative_y + PIX;
      pix[1][2] = 0.0f;
      pix[2][0] = relative_x + PIX;
      pix[2][1] = relative_y + PIX;
      pix[2][2] = 0.0f;
      pix[3][0] = relative_x + PIX;
      pix[3][1] = relative_y;
      pix[3][2] = 0.0f;
    }
  } else if (front_back == 1) {
    // 像素点位于朝向正面的三个面，正方形的顶点逆时针顺序排列
    if (direction == 0) {
      // 像素点位于垂直于X轴向前的平面（左前侧面）
      pix[0][0] = 1.0f;
      pix[0][1] = relative_x;
      pix[0][2] = relative_y;
      pix[1][0] = 1.0f;
      pix[1][1] = relative_x + PIX;
      pix[1][2] = relative_y;
      pix[2][0] = 1.0f;
      pix[2][1] = relative_x + PIX;
      pix[2][2] = relative_y + PIX;
      pix[3][0] = 1.0f;
      pix[3][1] = relative_x;
      pix[3][2] = relative_y + PIX;
    } else if (direction == 1) {
      // 像素点位于垂直于Y轴向前的平面（右前侧面）
      pix[0][0] = relative_y;
      pix[0][1] = 1.0f;
      pix[0][2] = relative_x;
      pix[1][0] = relative_y;
      pix[1][1] = 1.0f;
      pix[1][2] = relative_x + PIX;
      pix[2][0] = relative_y + PIX;
      pix[2][1] = 1.0f;
      pix[2][2] = relative_x + PIX;
      pix[3][0] = relative_y + PIX;
      pix[3][1] = 1.0f;
      pix[3][2] = relative_x;
    } else if (direction == 2) {
      // 像素点位于垂直于Z轴向上的平面（顶面）
      pix[0][0] = relative_x;
      pix[0][1] = relative_y;
      pix[0][2] = 1.0f;
      pix[1][0] = relative_x + PIX;
      pix[1][1] = relative_y;
      pix[1][2] = 1.0f;
      pix[2][0] = relative_x + PIX;
      pix[2][1] = relative_y + PIX;
      pix[2][2] = 1.0f;
      pix[3][0] = relative_x;
      pix[3][1] = relative_y + PIX;
      pix[3][2] = 1.0f;
    }
  }

  return pix;
}

void display_1(void) {
  // 设置逆时针排列的点围成的平面为正面
  glFrontFace(GL_CCW);
  // 设置不绘制背面，节省算力同时不会出现背面覆盖正面的情况
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  // 设置背景为白色
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT);
  // 加载单位阵
  glLoadIdentity();
  // 设置相机的位置和视角
  // 有关gluLookAt：https://blog.csdn.net/Augusdi/article/details/20470813
  gluLookAt(2, 2, 2, 0.0, 0.0, 0.0, -1, -1, 1);
  // 设置绕给定的轴旋转
  glTranslatef(axis[0][0], axis[0][1], axis[0][2]);
  glRotatef(angle, axis[1][0] - axis[0][0], axis[1][1] - axis[0][1],
            axis[1][2] - axis[0][2]);
  glTranslatef(-axis[0][0], -axis[0][1], -axis[0][2]);
  // 逐个像素点绘制小正方形
  for (unsigned int front_back = 0; front_back < 2; front_back++) {
    for (unsigned int direction = 0; direction < 3; direction++) {
      for (unsigned int i = 0; i < COLOR; i++) {
        for (unsigned int j = 0; j < COLOR; j++) {
          GLfloat** pix = get_pix(front_back, direction, (GLfloat)(i / COLOR),
                                  (GLfloat)(j / COLOR));
          glColor3fv(pix[0]);
          glBegin(GL_QUADS);
          for (unsigned int v = 0; v < 4; v++) glVertex3fv(pix[v]);
          glEnd();
        }
      }
    }
  }
  // 双缓冲下的刷新帧缓存
  glutSwapBuffers();
}

void display_2() {
  // 设置逆时针排列的点围成的平面为正面
  glFrontFace(GL_CCW);
  // 设置不绘制背面，节省算力同时不会出现背面覆盖正面的情况
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  // 设置背景为白色
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT);
  // 加载单位阵
  glLoadIdentity();
  // 设置相机的位置和视角
  // 有关gluLookAt：https://blog.csdn.net/Augusdi/article/details/20470813
  gluLookAt(2, 2, 2, 0.0, 0.0, 0.0, -1, -1, 1);
  // 设置绕给定的轴旋转
  glTranslatef(axis[0][0], axis[0][1], axis[0][2]);
  glRotatef(angle, axis[1][0] - axis[0][0], axis[1][1] - axis[0][1],
            axis[1][2] - axis[0][2]);
  glTranslatef(-axis[0][0], -axis[0][1], -axis[0][2]);
  // 设置立方体的八个顶点坐标
  static const GLfloat vertex[][3] = {
      0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f,
      0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
  // 设置绘制六个面时顶点的顺序
  static const GLint index[][4] = {0, 2, 3, 1, 0, 4, 6, 2, 0, 1, 5, 4,
                                   4, 5, 7, 6, 1, 3, 7, 5, 2, 6, 7, 3};
  // 绘制六个面
  glBegin(GL_QUADS);
  for (unsigned int i = 0; i < 6; i++)
    for (unsigned int j = 0; j < 4; j++) {
      // 每个顶点的RGB颜色值和其顶点位置坐标一致
      glColor3fv(vertex[index[i][j]]);
      glVertex3fv(vertex[index[i][j]]);
    }
  glEnd();
  // 双缓冲下的刷新帧缓存
  glutSwapBuffers();
}

// 动画所需的定时器回调函数
// 有关定时器回调函数：https://blog.csdn.net/shimazhuge/article/details/17894883
void timer_function(GLint value) {
  // 旋转角度增加
  angle += STEP;
  // 若角度大于360转完一圈则清零
  if (angle > 360.0) angle -= 360.0;
  glutPostRedisplay();
  glutTimerFunc(50, timer_function, value);
}

// 窗口大小自适应函数，使得窗口大小改变时仍保持图形的比例不变
// 有关窗口自适应函数：http://blog.sina.com.cn/s/blog_5497dc110102w8qh.html
void reshape(int w, int h) {
  glViewport(0, 0, (GLsizei)w, (GLsizei)h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 1.0, 20.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(2, 2, 2, 0.0, 0.0, 0.0, -1, -1, 1);
}

int main(int argc, char** argv) {
  glutInit(&argc, argv);
  // 设置双缓冲和RGB颜色模式
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
  // 设置窗口大小、位置和名称
  glutInitWindowSize(500, 500);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("color_cube");
  // 设置绘制函数、窗口大小自适应函数和定时器回调函数
  glutDisplayFunc(display_1);
  glutReshapeFunc(reshape);
  glutTimerFunc(500, timer_function, 1);
  // 进入主循环
  glutMainLoop();
  return 0;
}
