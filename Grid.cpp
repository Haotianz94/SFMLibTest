#include "Grid.h"

Grid::Grid( Point2f tl/*=Point2f(0,0)*/, Point2f dr/*=Point2f(1,1)*/ )
{
	m_tl = tl;
	m_dr = dr;
	m_xlen = m_dr.x-m_tl.x;
	m_ylen = m_dr.y-m_tl.y;
}

Grid::Grid( const Grid & rhs )
{
	m_tl = rhs.m_tl;
	m_dr = rhs.m_dr;
	m_xlen = rhs.m_xlen;
	m_ylen = rhs.m_ylen;
}

Grid::Grid( float x1, float x2, float y1, float y2 )
{
	m_tl = Point2f(x1, y1);
	m_dr = Point2f(x2, y2);
	m_xlen = x2-x1;
	m_ylen = y2-y1;
}

vector<double> Grid::getWeights( Point2f v )
{



	Vec2f pt = Vec2f(v.x, v.y);
	Vec2f V00_ = Vec2f(m_tl.x, m_tl.y);
	Vec2f V01_ = Vec2f(m_dr.x, m_tl.y);
	Vec2f V10_ = Vec2f(m_tl.x, m_dr.y);
	Vec2f V11_ = Vec2f(m_dr.x, m_dr.y);

	double a_x = V00_[0] - V01_[0] - V10_[0] + V11_[0];
	double b_x = -V00_[0] + V01_[0];
	double c_x = -V00_[0] + V10_[0];
	double d_x = V00_[0] - pt[0];

	double a_y = V00_[1]  - V01_[1] - V10_[1] + V11_[1];
	double b_y = -V00_[1] + V01_[1];
	double c_y = -V00_[1] + V10_[1];
	double d_y = V00_[1]  - pt[1];

	double bigA = -a_y*b_x + b_y*a_x;
	double bigB = -a_y*d_x - c_y*b_x + d_y*a_x +b_y*c_x;
	double bigC = -c_y*d_x + d_y*c_x;

	double tmp1 = -1;
	double tmp2 = -1;
	double tmp3 = -1;
	double tmp4 = -1;

	double k1 = 0,k2 = 0;

	if(bigB*bigB - 4*bigA*bigC >= 0.0)
	{
		if ( fabs(bigA) >= 0.000001)
		{
			tmp1 = ( -bigB + sqrt(bigB*bigB - 4*bigA*bigC) ) / ( 2*bigA );
			tmp2 = ( -bigB - sqrt(bigB*bigB - 4*bigA*bigC) ) / ( 2*bigA );
		}
		else
		{
			tmp1 = -bigC/bigB;
		}

		if ( tmp1 >= -0.999999 && tmp1 <= 1.000001)
		{
			tmp3 = -(b_y*tmp1 + d_y) / (a_y*tmp1 + c_y);
			tmp4 = -(b_x*tmp1 + d_x) / (a_x*tmp1 + c_x);
			if(tmp3 >= -0.999999 && tmp3 <= 1.000001)
			{
				k1 = tmp1;
				k2 = tmp3;
			}
			else if (tmp4 >= -0.999999 && tmp4 <= 1.000001)
			{
				k1 = tmp1;
				k2 = tmp4;
			}
		}

		if ( tmp2 >= -0.999999 && tmp2 <= 1.000001)
		{
			if ( tmp3 >= -0.999999 && tmp3 <= 1.000001)
			{
				k1 = tmp2;
				k2 = tmp3;
			}
			else if ( tmp4 >= -0.999999 && tmp4 <= 1.000001)
			{
				k1 = tmp2;
				k2 = tmp4;
			}
		}
	}

	vector<double> coefficients(4, -1);
	if ( k1>=-0.999999 && k1<=1.000001 && k2>=-0.999999 && k2<=1.000001)
	{
		coefficients[0] = (1.0-k1)*(1.0-k2);
		coefficients[2] = k1*(1.0-k2);
		coefficients[1] = (1.0-k1)*k2;
		coefficients[3] = k1*k2;

	}
	else{
		cout<<"fuck"<<endl;
	}
	return coefficients;
	
	
	vector<double> w;

	if(v.x+1e-8<m_tl.x||v.x>m_dr.x+1e-8||v.y+1e-8<m_tl.y||v.y>m_dr.y+1e-8){
		FOR(i,0,4)
			w.push_back(-1.f);
	}
	double xlenylen = m_xlen*m_ylen;
	w.push_back((m_dr.x-v.x)*(m_dr.y-v.y)/xlenylen);
	w.push_back((m_dr.x-v.x)*(v.y-m_tl.y)/xlenylen);
	w.push_back((m_dr.y-v.y)*(v.x-m_tl.x)/xlenylen);
	w.push_back((v.x-m_tl.x)*(v.y-m_tl.y)/xlenylen);
	return w;
}

Grid::~Grid()
{

}

Grid & Grid::operator=( const Grid & rhs )
{
	m_tl = rhs.m_tl;
	m_dr = rhs.m_dr;
	m_xlen = rhs.m_xlen;
	m_ylen = rhs.m_ylen;
	return *this;
}

