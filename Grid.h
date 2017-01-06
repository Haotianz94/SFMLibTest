#ifndef Grid_H_
#define Grid_H_
#include "global_header.h"
using namespace cv;
class Grid{

public:
	Grid(Point2f tl=Point2f(0,0), Point2f dr=Point2f(1,1));
	Grid(float x1, float x2, float y1, float y2);
	Grid (const Grid & rhs);
	Grid & operator = (const Grid & rhs);
	~Grid();
	vector<double> getWeights(Point2f v);
	inline Point2f getTL(){return m_tl;}
	inline Point2f getDR(){return m_dr;}
	inline Point2f getTR(){return Point2f(m_dr.x, m_tl.y);}
	inline Point2f getDL(){return Point2f(m_tl.x, m_dr.y);}
	//friend ostream& operator<<(ostream& out, const Grid& q);
	friend ostream& operator<<(ostream& out, const Grid& q){
		out<<"Grid: ["<<q.m_tl.x<<", "<<q.m_tl.y<<"] ["<<q.m_dr.x<<", "<<q.m_dr.y<<"]"<<endl;
		return out;
	}

private:
	Point2f m_tl, m_dr;
	float m_xlen;
	float m_ylen;
};



#endif


