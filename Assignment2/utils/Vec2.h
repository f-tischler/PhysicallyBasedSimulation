/******************************************************************
*
* Vec2.h
*
* Description: Code providing helper function for handling 2D
* vectors; standard operators are provided
*
* Physically-Based Simulation Proseminar WS 2015
*
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

#ifndef __VEC2_T_H__
#define __VEC2_T_H__

template<typename T>
class Vector2T
{
public:
    Vector2T() { m_xy[0] = (T)0; m_xy[1] = (T)0;}
    Vector2T(T v) { m_xy[0] = v; m_xy[1] = v; }
    Vector2T(const T x, const T y)  { m_xy[0] = x; m_xy[1] = y; }
    Vector2T(const Vector2T<T> &other) { *this = other; }
    
    Vector2T<T> & operator=( const Vector2T<T> &other ) 
    { 
        m_xy[0]= other.m_xy[0]; m_xy[1] = other.m_xy[1]; return *this; 
    }

    T& operator[](int i) { return m_xy[i]; }

    const T& x() const { return m_xy[0]; }
    const T& y() const { return m_xy[1]; }

    const T& operator[](int i) const { return m_xy[i]; }

    bool operator==(const Vector2T<T> &other) const
    {
        if(x()!=other.x()) return false;
        if(y()!=other.y()) return false;
        return true;
    }

    bool operator!=(const Vector2T<T> &other) const
    {
        return !(*this == other);
    }

    Vector2T<T> operator*( T s ) const
    {
        return Vector2T<T>(m_xy[0]*s, m_xy[1]*s);
    }

    Vector2T<T>& operator*=( T s )
    {
        m_xy[0] = m_xy[0] * s;
        m_xy[1] = m_xy[1] * s;
        return *this;
    }

    Vector2T<T> operator/( T s ) const
    {
        return Vector2T<T>(m_xy[0]/s, m_xy[1]/s);
    }

    Vector2T<T>& operator/=( T s )
    {
        m_xy[0] = m_xy[0] / s;
        m_xy[1] = m_xy[1] / s;
        return *this;
    }

    Vector2T<T> operator+( const Vector2T<T> &v2 ) const
    {
        return Vector2T<T>(x()+v2.x(),y()+v2.y());
    }

    Vector2T<T>& operator+=( const Vector2T<T> &v )
    {
        m_xy[0] += v.x();
        m_xy[1] += v.y();
        return *this;
    }

    Vector2T<T> operator-( const Vector2T<T> &v2 ) const
    {
        return Vector2T<T>(x()-v2.x(),y()-v2.y());
    }

    Vector2T<T>& operator-=( const Vector2T<T> &v )
    {
        m_xy[0] -= v.x();
        m_xy[1] -= v.y();
        return *this;
    }

    Vector2T<T> operator-() const
    {
        return Vector2T<T>(-m_xy[0], -m_xy[1]);
    }

    T operator|(const Vector2T<T> &v2) const
    {
        return (*this)[0] * v2[0] + (*this)[1] * v2[1];
    }

    Vector2T<T> normalized() const
    {
        return *this / length();
    }

    T norm() const
    {
        return length();
    }

    T sqrnorm() const
    {
        return squaredLength();
    }

    T length() const
    {
        return sqrt(x()*x() + y()*y());
    }

    T squaredLength() const
    {
        return (x()*x() + y()*y());
    }

    static int dim() { return 2; }

private:
    T m_xy[2];
};

template<class T>
Vector2T<T> operator*(T const &s, Vector2T<T> const &v)
{
    return v*s;
}

typedef Vector2T<double> Vector2;

#endif
