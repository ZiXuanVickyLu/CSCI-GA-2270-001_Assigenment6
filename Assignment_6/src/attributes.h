#pragma once

#include <Eigen/Core>
#include<math.h>
#include <iostream>
#include <utility>
const double pi = acos(-1.0);
class VertexAttributes
{
	public:
	VertexAttributes(float x = 0, float y = 0, float z = 0, float w = 1)
	{
		position << x,y,z,w;
		color << 0,0,0,1;
	}
	void set_position(const Eigen::Vector4f p){
	    position =  p;
	}
    void set_color(const Eigen::Vector4f c){
        color = c;
	}
    // Interpolates the vertex attributes
    static VertexAttributes interpolate(
        const VertexAttributes& a,
        const VertexAttributes& b,
        const VertexAttributes& c,
        const float alpha, 
        const float beta, 
        const float gamma
    ) 
    {
        VertexAttributes r;
        r.position = alpha*a.position + beta*b.position + gamma*c.position;
		r.color = alpha*a.color + beta*b.color + gamma*c.color;
        return r;
    }

    void interpolate_linear(
            const VertexAttributes& a,
            const VertexAttributes& b,
            float degree
            ){
        this -> position = (1-degree) * a.position + degree * b.position;
	}

	void interpolate_Bezier(
            const VertexAttributes& a,
            const VertexAttributes& b,
            const VertexAttributes& c,
            float degree
	        ){
            this ->position = 1 * pow((1 - degree), 2) * a.position +
             2 * (1-degree) * degree * b.position+
             1 * pow(degree,2) * c.position;
    }

    static inline int calcni( int n, int i){
	     int up = 1, down =1, down2 = 1;
        for(int j = 1;j<=n;j++)
            up *= j;
        for(int j = 1;j<=i;j++)
            down *= j;
        for(int j = 1;j<=(n-i);j++)
            down2 *= j;
        return (int) ((up/down)/down2);
	}

	Eigen::Vector4f position;
	Eigen::Vector4f color;

};


class FragmentAttributes
{
	public:
	FragmentAttributes(float r = 0, float g = 0, float b = 0, float a = 1)
	{
		color << r,g,b,a;
	}

	Eigen::Vector4f color;
};

class FrameBufferAttributes
{
	public:
	FrameBufferAttributes(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255)
	{
		color << r,g,b,a;
	}

	Eigen::Matrix<uint8_t,4,1> color;
};

class UniformAttributes
{
	public:
    UniformAttributes(){
        view.row(0) <<1,0,0,0;
        view.row(1) <<0,1,0,0;
        view.row(2) <<0,0,1,0;
        view.row(3) <<0,0,0,1;
        viewPort.row(0) <<1,0,0,0;
        viewPort.row(1) <<0,1,0,0;
        viewPort.row(2) <<0,0,1,0;
        viewPort.row(3) <<0,0,0,1;
    }

    void set_view_rotate(double theta){
        theta = theta * pi / 180.0;
        Eigen::Matrix<float,4,4> tmp;
        tmp.row(0) << cos(theta),-sin(theta),0,0;
        tmp.row(1) << sin(theta),cos(theta),0,0;
        tmp.row(2) << 0,0,1,0;
        tmp.row(3) << 0,0,0,1;
        view *= tmp;
    }
    void set_view_scale(float s){
        Eigen::Matrix<float,4,4> tmp;
        tmp.row(0) <<s,0,0,0;
        tmp.row(1) <<0,s,0,0;
        tmp.row(2) <<0,0,s,0;
        tmp.row(3) <<0,0,0,1;
        view *= tmp;

    }
    void set_view_tranlate(float p1, float p2){
        Eigen::Matrix<float,4,4> tmp;

        tmp.row(0) << 1,0,0,p1;
        tmp.row(1) << 0,1,0,p2;
        tmp.row(2) << 0,0,1,0;
        tmp.row(3) << 0,0,0,1;
        view *= tmp;
    }
    Eigen::Matrix<float,4,4> get_view() const {
        return view;
    }

    Eigen::Matrix<float,4,4> get_viewPort() const {
        return viewPort;
    }

    void set_viewPort(float w, float h){
        viewPort.row(0) << 1.0*h/w,0,0,0;
    }
    private:
        Eigen::Matrix<float,4,4> view;
        Eigen::Matrix<float,4,4> viewPort;
};