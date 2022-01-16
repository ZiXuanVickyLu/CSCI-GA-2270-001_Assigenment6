#include "SDLViewer.h"
#include <functional>
#include <iostream>
#include <stack>
#include "raster.h"
#include <vector>
#include <cmath>
#include <ctime>
#define FRAME_RATE 20
void Delay(double time)//time as second
 {
    clock_t now = clock();
    while((double)(clock() - now)/CLOCKS_PER_SEC < time)
        ;
}

extern const double pi;
Eigen:: Vector4f ColorMap[11];
std::vector<std::vector<VertexAttributes>>* keyframe_Buffer_vertex;
std::vector<std::vector<VertexAttributes>>* keyframe_Buffer_line;
void ColorInit(){
    ColorMap[0] << 0,0,0,1; //black
    ColorMap[1] << 1,0,0,1;
    ColorMap[2] << 0,1,0,1;
    ColorMap[3] << 0,0,1,1;
    ColorMap[4] << 1,1,0,1;
    ColorMap[5] << 0,1,1,1;
    ColorMap[6] << 1,0,1,1;
    ColorMap[7] << 1,0.5,0,1;
    ColorMap[8] << 1,0,0.5,1;
    ColorMap[9] << 0.5,1,0.5,1;
}
//vertex of the triangles
std::vector<VertexAttributes> vertices;

std::vector<VertexAttributes> line;

Eigen::Vector4f truthPos(const float x, const float y,const UniformAttributes &u){
    Eigen::Vector4f at;
    auto vi = u.get_view();
    auto vip = u.get_viewPort();
    at << x/vip.row(0)[0],y,0,1;
    at = vi.inverse() * at;
    return at;
}

class triangle{
public:
    triangle(int v1_, int v2_, int v3_):v1(v1_), v2(v2_), v3(v3_){
        this -> calculateCenter();
        this -> is_selected = false;
    }
    void after_insert(){
        vertices[v1].color = ColorMap[1];
        vertices[v2].color = ColorMap[1];
        vertices[v3].color = ColorMap[1];

        for(int i = 0;i<3;i++)
            colorRecord[i] = ColorMap[1];

        this -> calculateCenter();
    }
    void set_triangle_vertex(int v1_, int v2_, int v3_){
        this -> v1 = v1_;
        this -> v2 = v2_;
        this -> v3 = v3_;

        this -> calculateCenter();
    }
    int in_vertex(float p1, float p2,UniformAttributes & u){
        if(is_delete)
            return false;
        auto at = truthPos(p1,p2,u);
        Eigen::Vector2f a;
        a << at[0],at[1];
        for(int i = 0;i<3;i++) {
            if (sqrt(pow((at[0]-this -> visit012(i).position[0]),2)+pow((at[1]-this -> visit012(i).position[1]),2))<0.04)
            {is_selected = !is_selected; return i+1;}
        }
        return 0;
    }

    bool in_triangle(float p1, float p2,UniformAttributes & u){
        if(is_delete)
            return false;
        auto at = truthPos(p1,p2,u);
        Eigen::Vector2f a;
        a << at[0],at[1];
        Eigen::Vector2f edge;
        Eigen::Vector2f point;
        float res[3];
        for(int i = 0;i<3;i++){
            auto tmp = - this ->visit012(i%3).position + this->visit012((i+1)%3).position;
            auto tmp2 = - this ->visit012(i%3).position;
            edge << tmp[0],tmp[1];
            point << (tmp2[0] + a[0]), (tmp2[1]+a[1]);
            res[i] =  this->outer(edge,point);
        }
        if((res[0]>=0 && res[1]>=0 && res[2]>=0) || (res[0]<=0 && res[1]<=0 && res[2]<=0)){
            is_selected = !is_selected;
            return true;
        }
        return false;
    }

    VertexAttributes visit012(int i) const{
        switch(i){
            case 0:
                return vertices[v1];break;
            case 1:
                return vertices[v2];break;
            case 2:
                return vertices[v3];break;
        }
    }

    void visit012SetPosition(int i, Eigen::Vector4f position){
        switch(i){
            case 0:
                vertices[v1].position = position; break;
            case 1:
                vertices[v2].position = position; break;
            case 2:
                vertices[v3].position = position; break;
        }

    }
    void set_selectColor(){

        colorRecord[0] = vertices[v1].color;
        colorRecord[1] = vertices[v2].color;
        colorRecord[2] = vertices[v3].color;

        vertices[v1].color << 0,0,1,1;
        vertices[v2].color << 0,0,1,1;
        vertices[v3].color << 0,0,1,1;
        center.color <<0,0,1,1;
        is_selected = true;
    }
    void set_unselectColor(){
        vertices[v1].color = colorRecord[0];
        vertices[v2].color = colorRecord[1];
        vertices[v3].color = colorRecord[2];
        center.color = (vertices[v1].color +
                        vertices[v2].color +
                        vertices[v3].color)/3.0;
        is_selected = false;
    }
    void visit012SetColor(int i, Eigen::Vector4f color){
        switch(i){
            case 0:
                colorRecord[0] = color; break;
            case 1:
                colorRecord[1] = color; break;
            case 2:
                colorRecord[2] = color; break;
        }
    }
    void delet(){
        is_delete = true;
        Eigen::Vector4f c;
        c<< 1,1,1,0;
        for(int i = 0;i<3;i++)
            visit012SetColor(i,c);
        this ->set_unselectColor();
        this -> set_triangle_scale(0.001);
        this -> set_triangle_translate(-1.5,-1.5);
        this ->changeline();

    }

    bool isSelected(){
        return is_selected;
    }
    void setCancel(){
        this -> is_selected = false;
        this -> set_unselectColor();
    }
    void set_triangle_rotate(double theta){
        Eigen::Matrix<float,2,2> r;
        theta = theta*pi/180;
        r.row(0) << cos(theta),-sin(theta);
        r.row(1) << sin(theta),cos(theta);
        Eigen::Vector2f e[3];
        for (int i = 0; i < 3; i++){
            auto tmp = this -> visit012(i).position-center.position;
            e[i] << tmp[0],tmp[1];
            e[i] = r * e[i];
            auto v = this -> visit012(i).position;
            v[0] = e[i][0],v[1] = e[i][1];
            v += center.position, v[3] = 1;
            visit012SetPosition(i,v);
        }
        this ->changeline();
    }

    void set_triangle_scale(double s){
        if (s<1e-6)
            return; // s==0, error
        Eigen::Matrix<float,2,2> sc;
        sc.row(0) << s,0;
        sc.row(1) << 0,s;
        Eigen::Vector2f e[3];
        for (int i = 0; i < 3; i++){
            auto tmp = this -> visit012(i).position-center.position;
            e[i] << tmp[0],tmp[1];
            e[i] = sc * e[i];
            auto v = this -> visit012(i).position;
            v[0] = e[i][0],v[1] = e[i][1];
            v += center.position, v[3] = 1;
            visit012SetPosition(i,v);
        }
        this -> changeline();

    }

    void set_triangle_translate(float p1, float p2){
        Eigen::Matrix<float,4,4> tr;
        tr.row(0) << 1,0,0,p1;
        tr.row(1) << 0,1,0,p2;
        tr.row(2) << 0,0,1,0;
        tr.row(3) << 0,0,0,1;
        this -> calcToCenter(tr);
        this -> calculateCenter();
        this ->changeline();
    }

    void changePosByCenter(const float x, const float y,UniformAttributes &u){
        auto a = truthPos(x, y,u);
        Eigen::Vector4f e[3];
        for (int i = 0; i < 3; i++){
            e[i] = this -> visit012(i).position - this -> center.position; //vector from center towards vi
            e[i][0] += a[0], e[i][1] += a[1];
            e[i][3] = 1;
            visit012SetPosition(i,e[i]);
        }
        this -> calculateCenter();
        this ->changeline();
    }

private:
    int v1, v2, v3;
    Eigen::Vector4f colorRecord[3];
    VertexAttributes center;
    bool is_selected ;
    bool is_delete = false;

    void  visitlineSetPosition(int i, Eigen::Vector4f position) const{
        line[2*v1+i].position = position;
    }
    void  visitlineSetColor(int i, Eigen::Vector4f c) const{
        line[2*v1+i].color = c;
    }
    void changeline() {
        if(is_delete){
            Eigen::Vector4f c;
            c << 1,1,1,0;
            for(int i = 0;i<6;i++)
                visitlineSetColor(i,c);
        }
        visitlineSetPosition(0,vertices[v3].position);
        visitlineSetPosition(1,vertices[v2].position);
        visitlineSetPosition(2,vertices[v2].position);
        visitlineSetPosition(3,vertices[v1].position);
        visitlineSetPosition(4,vertices[v1].position);
        visitlineSetPosition(5,vertices[v3].position);
    }

    void calcToCenter(const Eigen::Matrix<float,4,4> &m){
        Eigen::Vector4f e[3];
        for (int i = 0; i < 3; i++){
            e[i] = this -> visit012(i).position;
            e[i] = m * e[i];
            visit012SetPosition(i,e[i]);
        }
    }
    void calculateCenter(){
        center.position = (vertices[v1].position + vertices[v2].position + vertices[v3].position)/3.0;
        center.color = (vertices[v1].color + vertices[v2].color + vertices[v3].color)/3.0;
    }

    float outer(const Eigen::Vector2f &e1,const Eigen::Vector2f &e2)const{
        return e1[0]*e2[1]-e1[1]*e2[0];
    }
};

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"


/*--------------------------------------global struct--------------------------------------------------*/
//triangle buffer: triangle={index1,2,3}, index->vertex on 'vertices'
std::vector <triangle> triangles;

//temp line for the insert method
std::stack<VertexAttributes> tempLineVertex;
std::vector<VertexAttributes> LineBuffer;

/*--------------------------------------end global struct--------------------------------------------------*/
void calc_play_keyframe_Linear(SDLViewer &viewer){
    int len = keyframe_Buffer_line->size(); //which keyframe
    int  play_time = len-1;
    int triangles_n = triangles.size();
    float gap = 1.0/FRAME_RATE;

    for(int time = 0; time < play_time; time++){
        float index = 0;
        while(index <=1.0) {
            for (int j = 0; j < triangles_n; j++){
                for(int v = 0;v<3;v++)
                    vertices[j*3+v].interpolate_linear(keyframe_Buffer_vertex->at(time)[j*3+v],
                                                       keyframe_Buffer_vertex->at(time+1)[j*3+v],index);
                for(int l = 0;l<6;l++)
                    line[j*6+l].interpolate_linear(keyframe_Buffer_line->at(time)[j*6+l],
                                                       keyframe_Buffer_line->at(time+1)[j*6+l],index);
            }//end one frame's calc
            index += gap;
            viewer.redraw(viewer);
        }
    }
}

void calc_play_keyframe_Bezier(SDLViewer &viewer){
//len = 3
    int triangles_n = triangles.size();
    float gap = 1.0/FRAME_RATE;
    float index = 0;
    while(index <= 1) {
            for (int j = 0; j < triangles_n; j++){
                for(int v = 0;v<3;v++)
                    vertices[j*3+v].interpolate_Bezier(keyframe_Buffer_vertex->at(0)[j*3+v],
                                                       keyframe_Buffer_vertex->at(1)[j*3+v],
                                                       keyframe_Buffer_vertex->at(2)[j*3+v],
                                                       index);
                for(int l = 0;l<6;l++)
                    line[j*6+l].interpolate_Bezier(keyframe_Buffer_line->at(0)[j*6+l],
                                                       keyframe_Buffer_line->at(1)[j*6+l],
                                                       keyframe_Buffer_line->at(2)[j*6+l],
                                                       index);
            }//end one frame's calc
        index += gap;
        viewer.redraw(viewer);
    }

}

/*------------------------------------------------main--------------------------------------------------*/
int main(int argc, char *args[])
{
    int width = 800;
    int height = 400;
    ColorInit();
    // The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer(width, height);

	// Global Constants (empty in this example)
	UniformAttributes uniform;
    uniform.set_viewPort(width,height);//viewPort translation

	// Basic rasterization program

	Program program;
    SDLViewer viewer;

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
    {
        VertexAttributes tmp;
        tmp.set_position ( uniform.get_viewPort() * uniform.get_view() * va.position);
        tmp.set_color(va.color);
	    return tmp;
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		return FragmentAttributes(va.color(0),va.color(1),va.color(2));
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		return FrameBufferAttributes(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
	};

    // Initialize the viewer and the corresponding callbacks

    viewer.init("triangle editor", width, height);

    viewer.clean = [&](){
        delete keyframe_Buffer_vertex;
        delete keyframe_Buffer_line;
        viewer.set_Mode(NORM);
    };
   /*
    viewer.lineplay = [&](int* i){

        int len = keyframe_Buffer_line->size(); //which keyframe
        int  play_time = len - 1 - (int)(*(i)/FRAME_RATE);
        int triangles_n = triangles.size();
        float gap = (float)1.0/FRAME_RATE/len;
            int n = len * FRAME_RATE - *i - 1 - play_time * FRAME_RATE;
            float index =  gap * (float)n;
                for (int j = 0; j < triangles_n; j++){
                    for(int v = 0;v<3;v++)
                        vertices[j*3+v].interpolate_linear(keyframe_Buffer_vertex->at(play_time)[j*3+v],
                                                           keyframe_Buffer_vertex->at(play_time+1)[j*3+v],index);
                    for(int l = 0;l<6;l++)
                        line[j*6+l].interpolate_linear(keyframe_Buffer_line->at(play_time)[j*6+l],
                                                       keyframe_Buffer_line->at(play_time+1)[j*6+l],index);
                }//end one frame's calc
                viewer.redraw_next = true;
                viewer.redraw(viewer);
                Delay(2);
                viewer.add_line_event((*i)-1);
    };
*/
    viewer.mouse_move = [&](int x, int y, int xrel, int yrel){
        switch(viewer.getMode()) {
            case INSERT1:
                LineBuffer[1].position = truthPos((float(x) / float(width) * 2) - 1, (float(height - 1 - y) / float(height) * 2) - 1,
                                                  uniform);
                viewer.redraw_next = true;
                break;
            case INSERT2:

                LineBuffer[1].position = truthPos((float(x) / float(width) * 2) - 1, (float(height - 1 - y) / float(height) * 2) - 1,
                                                  uniform);
                LineBuffer[2].position = truthPos((float(x) / float(width) * 2) - 1, (float(height - 1 - y) / float(height) * 2) - 1,
                                                  uniform);
                viewer.redraw_next = true;
                break;

                case HOLD_ON: case KEYFRAME_HOLD_ON: {
                    if (viewer.get_active() >= 0) {
                        if (viewer.getMode() == HOLD_ON || viewer.getMode() == KEYFRAME_HOLD_ON) {
                            triangles[viewer.get_active()].changePosByCenter
                                    ((float(x) / float(width) * 2) - 1, (float(height - 1 - y) / float(height) * 2) - 1,
                                     uniform);
                        }
                        viewer.redraw_next = true;
                    }
                }
        }
    };

    viewer.mouse_pressed = [&](int x, int y, bool is_pressed, int button, int clicks) {
        switch(viewer.getMode()) {
            case DELETE:
                if(is_pressed) {
                    for (int i = 0; i < triangles.size(); i++) {
                        if (triangles[i].in_triangle((float(x) / float(width) * 2) - 1,
                                                     (float(height - 1 - y) / float(height) * 2) - 1, uniform)){
                            triangles[i].delet();
                            auto tb = triangles.begin();
                            viewer.redraw_next = true;
                            break;
                        }
                    }
                }
                break;
            case INSERT_ON:
                if(is_pressed){
                    VertexAttributes now;
                    now.color = ColorMap[0];
                    now.position = truthPos(
                            (float(x) / float(width) * 2) - 1,(float(height - 1 - y) / float(height) * 2) - 1,uniform);
                    tempLineVertex.push(now);
                    LineBuffer.push_back(now);
                    LineBuffer.push_back(now);
                    viewer.set_Mode(INSERT1);
                }
                break;
            case INSERT1:
                if(is_pressed){
                    VertexAttributes now;
                    now.color = ColorMap[0];
                    now.position = truthPos(
                            (float(x) / float(width) * 2) - 1,(float(height - 1 - y) / float(height) * 2) - 1,uniform);
                    LineBuffer.clear();

                    LineBuffer.push_back(tempLineVertex.top());
                    LineBuffer.push_back(now);
                    LineBuffer.push_back(now);
                    LineBuffer.push_back(now);

                    line.push_back(tempLineVertex.top()); //last v1
                    line.push_back(now); //this v2
                    tempLineVertex.push(now);
                    viewer.set_Mode(INSERT2);
                    viewer.redraw_next = true;
                }
                break;

            case INSERT2:
                if(is_pressed){
                    VertexAttributes now;
                    now.color = ColorMap[0]; //black
                    int len = vertices.size();
                    now.position = truthPos(
                            (float(x) / float(width) * 2) - 1,(float(height - 1 - y) / float(height) * 2) - 1,uniform);
                    vertices.push_back(now);
                    while(!tempLineVertex.empty()) {
                        vertices.push_back(tempLineVertex.top());
                        tempLineVertex.pop();
                    }

                    line.push_back(vertices[len+1]);
                    line.push_back(vertices[len]);
                    line.push_back(vertices[len]);
                    line.push_back(vertices[len+2]);

                    LineBuffer.clear();
                    triangle newTriangle(len,len+1,len+2);
                    newTriangle.after_insert();
                    triangles.push_back(newTriangle);
                    viewer.set_Mode(INSERT_ON);
                    viewer.redraw_next = true;
                }
                break;
            case HOLD_ON: case KEYFRAME_HOLD_ON:{
                if(!is_pressed){ //release
                    if(viewer.getMode() == HOLD_ON)
                        viewer.set_Mode(TRANSLATION);
                    else
                        viewer.set_Mode(KEYFRAME_TRANSLATION);

                }
                break;
            }
            case COLOR: {

                if(is_pressed){
                    bool set = false;
                for (int i = 0; i < triangles.size(); i++){
                    int t = 0;
                    t = triangles[i].in_vertex((float(x) / float(width) * 2) - 1,
                                               (float(height - 1 - y) / float(height) * 2) - 1, uniform);
                    if (t) {
                        if (triangles[i].isSelected()) {
                            triangles[i].set_selectColor();
                            if (viewer.get_active() >= 0) {
                                triangles[viewer.get_active()].set_unselectColor();
                            } //already active, set inactive first)
                            viewer.set_active(i),viewer.set_vertex(t-1);
                        } else
                            triangles[i].set_unselectColor(),
                                    viewer.set_inactive();
                        viewer.redraw_next = true;
                        break;
                    }
                }
              }
                break;
            }
            case TRANSLATION: case KEYFRAME_TRANSLATION:{
                if(is_pressed) {
                    for (int i = 0; i < triangles.size(); i++) {
                        if (triangles[i].in_triangle((float(x) / float(width) * 2) - 1,
                                                     (float(height - 1 - y) / float(height) * 2) - 1,uniform)) {
                            if (viewer.getMode() == TRANSLATION)
                                viewer.set_Mode(HOLD_ON);
                            else
                                viewer.set_Mode(KEYFRAME_HOLD_ON);

                            if (triangles[i].isSelected()) {
                                triangles[i].set_selectColor();
                                if (viewer.get_active() >= 0) {
                                    triangles[viewer.get_active()].set_unselectColor();
                                } //already active, set inactive first)
                                viewer.set_active(i);
                            } else
                                triangles[i].set_unselectColor(),
                                        viewer.set_inactive();
                            viewer.redraw_next = true;
                            break;//break for
                        }
                    }
                }
                break;
            }
        }
    };

    viewer.mouse_wheel = [&](int dx, int dy, bool is_direction_normal) {
    };

    viewer.key_pressed = [&](char key, bool is_pressed, int modifier, int repeat) {
        if(is_pressed){

            switch(key){
                case 'i':
                    if(viewer.getMode() == KEYFRAME_START||viewer.getMode() == KEYFRAME_TRANSLATION)
                        delete keyframe_Buffer_vertex,
                        delete keyframe_Buffer_line;
                    viewer.set_Mode(INSERT_ON);
                    break;
                case 'o':
                    if(viewer.getMode() == KEYFRAME_START)
                        viewer.set_Mode(KEYFRAME_TRANSLATION);
                    else
                        viewer.set_Mode(TRANSLATION);
                    break;
                case 'p':
                        if(viewer.getMode() == KEYFRAME_START||viewer.getMode() == KEYFRAME_TRANSLATION)
                            delete keyframe_Buffer_vertex,
                            delete keyframe_Buffer_line;
                        viewer.set_Mode(DELETE);
                    break;
                case 'h':
                    if((viewer.getMode() == TRANSLATION || viewer.getMode() == KEYFRAME_TRANSLATION) && viewer.get_active()>=0)
                        triangles[viewer.get_active()].set_triangle_rotate(-10),
                        viewer.redraw_next = true;
                    break;
                case 'j':
                    if((viewer.getMode() == TRANSLATION || viewer.getMode() == KEYFRAME_TRANSLATION) && viewer.get_active()>=0)
                        triangles[viewer.get_active()].set_triangle_rotate(10),
                        viewer.redraw_next = true;
                    break;
                case 'k':
                    if((viewer.getMode() == TRANSLATION || viewer.getMode() == KEYFRAME_TRANSLATION) && viewer.get_active()>=0)
                        triangles[viewer.get_active()].set_triangle_scale(1.25),
                        viewer.redraw_next = true;
                    break;
                case 'l':
                    if((viewer.getMode() == TRANSLATION || viewer.getMode() == KEYFRAME_TRANSLATION) && viewer.get_active()>=0)
                        triangles[viewer.get_active()].set_triangle_scale(0.75),
                        viewer.redraw_next = true;
                    break;
                case '1':
                    if(viewer.getMode()==COLOR && viewer.get_active()>=0)
                        triangles[viewer.get_active()].visit012SetColor(viewer.get_vertex(),ColorMap[1]),
                        viewer.redraw_next = true;
                    break;
                case '2':
                    if(viewer.getMode()==COLOR && viewer.get_active()>=0)
                        triangles[viewer.get_active()].visit012SetColor(viewer.get_vertex(),ColorMap[2]),
                                viewer.redraw_next = true;
                    break;
                case '3':
                    if(viewer.getMode()==COLOR && viewer.get_active()>=0)
                        triangles[viewer.get_active()].visit012SetColor(viewer.get_vertex(),ColorMap[3]),
                                viewer.redraw_next = true;
                    break;
                case '4':
                    if(viewer.getMode()==COLOR && viewer.get_active()>=0)
                        triangles[viewer.get_active()].visit012SetColor(viewer.get_vertex(),ColorMap[4]),
                                viewer.redraw_next = true;
                    break;
                case '5':
                    if(viewer.getMode()==COLOR && viewer.get_active()>=0)
                        triangles[viewer.get_active()].visit012SetColor(viewer.get_vertex(),ColorMap[5]),
                                viewer.redraw_next = true;
                    break;
                case '6':
                    if(viewer.getMode()==COLOR && viewer.get_active()>=0)
                        triangles[viewer.get_active()].visit012SetColor(viewer.get_vertex(),ColorMap[6]),
                                viewer.redraw_next = true;
                    break;
                case '7':
                    if(viewer.getMode()==COLOR && viewer.get_active()>=0)
                        triangles[viewer.get_active()].visit012SetColor(viewer.get_vertex(),ColorMap[7]),
                                viewer.redraw_next = true;
                    break;
                case '8':
                    if(viewer.getMode()==COLOR && viewer.get_active()>=0)
                        triangles[viewer.get_active()].visit012SetColor(viewer.get_vertex(),ColorMap[8]),
                                viewer.redraw_next = true;
                    break;
                case '9':
                    if(viewer.getMode()==COLOR && viewer.get_active()>=0)
                        triangles[viewer.get_active()].visit012SetColor(viewer.get_vertex(),ColorMap[9]),
                                viewer.redraw_next = true;
                    break;
                case 'c':
                    if(viewer.getMode() == KEYFRAME_START||viewer.getMode() == KEYFRAME_TRANSLATION)
                        delete keyframe_Buffer_vertex,
                        delete keyframe_Buffer_line;
                    viewer.set_Mode(COLOR);
                    break;
                case 'a':
                    uniform.set_view_tranlate(0.2,0);
                    viewer.redraw_next = true;
                    break;
                case 'w':
                    uniform.set_view_tranlate(0,-0.2);
                    viewer.redraw_next = true;
                    break;
                case 's':
                    uniform.set_view_tranlate(0,0.2);
                    viewer.redraw_next = true;
                    break;
                case 'd':
                    uniform.set_view_tranlate(-0.2,0);
                    viewer.redraw_next = true;
                    break;
                case '=':
                    uniform.set_view_scale(1.1);
                    viewer.redraw_next = true;
                    break;
                case '-':
                    uniform.set_view_scale(0.9);
                    viewer.redraw_next = true;
                    break;
                case 'v': //keyframe
                    if(viewer.getMode()!= KEYFRAME_START && viewer.getMode()!=KEYFRAME_TRANSLATION){
                        viewer.set_Mode(KEYFRAME_START);
                        keyframe_Buffer_vertex = new std::vector<std::vector<VertexAttributes>>;
                        keyframe_Buffer_line = new std::vector<std::vector<VertexAttributes>>;
                    }
                    if(viewer.getMode()==KEYFRAME_TRANSLATION)
                        viewer.set_Mode(KEYFRAME_START);
                    keyframe_Buffer_vertex->push_back(vertices);
                    keyframe_Buffer_line->push_back(line);
                    break;
                case 'n': //linear
                    if(viewer.getMode() == KEYFRAME_START)
                        calc_play_keyframe_Linear(viewer),
                        viewer.clean();
                        /*viewer.add_line_event(FRAME_RATE*keyframe_Buffer_vertex->size());*/
                    break;
                case 'm': //cancel
                    if(viewer.getMode() == KEYFRAME_START||viewer.getMode() == KEYFRAME_TRANSLATION)
                        vertices = keyframe_Buffer_vertex->at(0),
                        line = keyframe_Buffer_line->at(0),
                        delete keyframe_Buffer_vertex,
                        delete keyframe_Buffer_line,
                        viewer.redraw_next = true,
                        viewer.set_Mode(NORM);
                    break;
                case 'b': //bezier
                    if(viewer.getMode() == KEYFRAME_START) {
                        if (keyframe_Buffer_vertex->size() == 3)
                            calc_play_keyframe_Bezier(viewer);
                        else
                            std::cout << "KEYFRAME_NUM_ERROE:: should have 3 keyframe."<< std::endl;
                        viewer.clean();
                    }
                        /*viewer.add_Bezier_event(FRAME_RATE*keyframe_Buffer_vertex->size());*/
                    break;
                case ' ':
                    if(viewer.getMode() == KEYFRAME_START||viewer.getMode() == KEYFRAME_TRANSLATION)
                        viewer.clean();
                    else
                        viewer.set_Mode(NORM);
                    for(int i = 0;i<triangles.size();i++)
                        triangles[i].setCancel();
                    viewer.set_inactive();
                    viewer.redraw_next = true;
                    break;
                case ']':
                    viewer.dump();
                    break;
            }
        }
    };

    viewer.redraw = [&](SDLViewer &viewer) {
        // Clear the framebuffer

        for (unsigned i=0;i<frameBuffer.rows();i++)
            for (unsigned j=0;j<frameBuffer.cols();j++)
                frameBuffer(i,j).color << 255,255,255,255;


        if(!vertices.empty())
       	    rasterize_triangles(program,uniform,vertices,frameBuffer);
       	if(!line.empty() && line.size()%2==0)
            rasterize_lines(program,uniform,line, 1,frameBuffer);
        if(!LineBuffer.empty() && LineBuffer.size()%2==0)
            rasterize_lines(program,uniform,LineBuffer, 1,frameBuffer);
        // Buffer for exchanging data between rasterizer and sdl viewer
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> R(width, height);
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> G(width, height);
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> B(width, height);
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> A(width, height);

        for (unsigned i=0; i<frameBuffer.rows();i++)
        {
            for (unsigned j=0; j<frameBuffer.cols();j++)
            {
                R(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(0);
                G(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(1);
                B(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(2);
                A(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(3);
            }
        }
        viewer.draw_image(R, G, B, A);
    };

    viewer.launch(1);

    return 0;
}
