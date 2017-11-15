/******************************************************************
*
* Vec3.h
*
* Description: Code providing helper function for handling 3D
* vectors; standard operators are provided
*
* Physically-Based Simulation Proseminar WS 2015
*
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

#ifndef __VEC3_T_H__
#define __VEC3_T_H__

#include <limits>
using namespace std;

template<typename T>
class Vector3T
{
public:
    Vector3T() { m_xyz[0] = (T)0; m_xyz[1] = (T)0; m_xyz[2] = (T)0;}
    explicit Vector3T(T v) { m_xyz[0] = v; m_xyz[1] = v; m_xyz[2] = v;}
    Vector3T(T x, T y, T z)  { m_xyz[0] = x; m_xyz[1] = y; m_xyz[2] = z; }
    explicit Vector3T(const T* xyz) { m_xyz[0] = xyz[0]; m_xyz[1] = xyz[1]; m_xyz[2] = xyz[2]; }
    Vector3T(const Vector3T<T> &other) { *this = other; }

    template <typename Scalar2>
    explicit Vector3T(const Vector3T<Scalar2> &m) 
    {
        for(int r=0; r<m.dim(); r++)
        {
            (*this)[r] = static_cast<T> (m[r]);
        }
    }

    Vector3T<T> & operator=( const Vector3T<T> &other ) 
    { 
        m_xyz[0]= other.m_xyz[0]; 
        m_xyz[1] = other.m_xyz[1]; 
        m_xyz[2] = other.m_xyz[2]; 
        return *this; 
    }

    T& operator[](int i) { return m_xyz[i]; }
    T& x() { return m_xyz[0]; }
    T& y() { return m_xyz[1]; }
    T& z() { return m_xyz[2]; }

    const T& operator[](int i) const { return m_xyz[i]; }
    const T& x() const { return m_xyz[0]; }
    const T& y() const { return m_xyz[1]; }
    const T& z() const { return m_xyz[2]; }

    void setX(T x) { m_xyz[0] = x; }
    void setY(T y) { m_xyz[1] = y; }
    void setZ(T z) { m_xyz[2] = z; }

    void xyz(T* xyz) const { xyz[0] = m_xyz[0]; xyz[1] = m_xyz[1]; xyz[2] = m_xyz[2]; }
    void setXYZ(const T* xyz) { m_xyz[0] = xyz[0]; m_xyz[1] = xyz[1]; m_xyz[2] = xyz[2]; }

    bool isNullVector() const { return (m_xyz[0] == (T)0) && (m_xyz[1] == (T)0) && (m_xyz[2] == (T)0); }

    bool operator==(const Vector3T<T> &other) const 
    {
        if(x()!=other.x()) return false;
        if(y()!=other.y()) return false;
        if(z()!=other.z()) return false;
        return true;
    }

    bool operator!=(const Vector3T<T> &other) const 
    {
        return !(*this == other);
    }

    bool operator<(const Vector3T<T> &other) const 
    {
        for(int i=0; i<3; i++)
        {
            if((*this)[i] != other[i])
                return (*this)[i] < other[i];
        }

        return false;
    }

    bool operator>(const Vector3T<T> &other) const 
    {
        for(int i=0; i<3; i++)
        {
            if((*this)[i] != other[i])
                return (*this)[i] > other[i];
        }

        return false;
    }

    Vector3T<T> operator*( T s ) const 
    {
        return Vector3T<T>(m_xyz[0]*s, m_xyz[1]*s, m_xyz[2]*s);
    }

    Vector3T<T>& operator*=( T s ) 
    {
        m_xyz[0] *= s;
        m_xyz[1] *= s;
        m_xyz[2] *= s;
        return *this;
    }

    Vector3T<T> operator/( T s ) const 
    {
        return Vector3T<T>(m_xyz[0]/s, m_xyz[1]/s, m_xyz[2]/s);
    }

    Vector3T<T>& operator/=( T s ) 
    {
        m_xyz[0] /= s;
        m_xyz[1] /= s;
        m_xyz[2] /= s;
        return *this;
    }

    Vector3T<T> operator+( const Vector3T<T> &v2 ) const 
    {
        return Vector3T<T>(x()+v2.x(),y()+v2.y(),z()+v2.z());
    }

    Vector3T<T>& operator+=( const Vector3T<T> &v ) 
    {
        m_xyz[0] += v.x();
        m_xyz[1] += v.y();
        m_xyz[2] += v.z();
        return *this;
    }

    Vector3T<T> operator-( const Vector3T<T> &v2 ) const 
    {
        return Vector3T<T>(x()-v2.x(),y()-v2.y(),z()-v2.z());
    }

    Vector3T<T>& operator-=( const Vector3T<T> &v ) 
    {
        m_xyz[0] -= v.x();
        m_xyz[1] -= v.y();
        m_xyz[2] -= v.z();
        return *this;
    }

    Vector3T<T> operator-() const 
    {
        return Vector3T<T>(-m_xyz[0], -m_xyz[1], -m_xyz[2]);
    }

    Vector3T<T> componentWiseProduct(const Vector3T<T> &v2) const 
    {
        return Vector3T<T>(x()*v2.x(), y()*v2.y(), z()*v2.z());
    }

    Vector3T<T> componentWiseDivision(const Vector3T<T> &v2) const 
    {
        return Vector3T<T>(x()/v2.x(), y()/v2.y(), z()/v2.z());
    }

    T operator*(  const Vector3T<T> &v2 ) const 
    {
        return x()*v2.x()+y()*v2.y()+z()*v2.z();
    }

    T operator|(const Vector3T<T> &v2) const 
    {
        return (*this) * v2;
    }
    
    Vector3T<T> cross( const Vector3T<T> &v2 ) const 
    {
        return Vector3T<T>(y()*v2.z()-v2.y()*z(), 
                           z()*v2.x()-v2.z()*x(), 
                           x()*v2.y()-v2.x()*y() );
    }

    Vector3T<T> operator%(const Vector3T<T> &v2) const 
    {
        return cross(v2);
    }

    void normalize() 
    {
        *this /= length();
    }

    Vector3T<T> normalized() const 
    {
        return *this / length();
    }

    T length() const 
    {
        return sqrt(squaredLength());
    }

    T norm() const 
    {
        return length();
    }

    T squaredLength() const
    {
        return m_xyz[0]*m_xyz[0]+m_xyz[1]*m_xyz[1]+m_xyz[2]*m_xyz[2];
    }

    T sqrnorm() const 
    {
        return squaredLength();
    }

    Vector3T<T> orthogonalVector() const 
    {
        Vector3T<T> vecNorm = normalized();

        const int dim = 3;

        int minIdx = -1;
        T minDot = 0;

        for(int i=0; i<dim; i++)
        {
            Vector3T<T> basisVec;
            for(int j=0; j<dim; j++)
                basisVec[j] = (j==i);

            T dotProd = vecNorm | basisVec;

            if(minIdx == -1 || fabs(dotProd) < fabs(minDot))
            {
                minIdx = i;
                minDot = dotProd;
            }
        }

        Vector3T<T> basisVec;
        for(int j=0; j<dim; j++)
            basisVec[j] = (j==minIdx);

        return basisVec - vecNorm * minDot;
    }

    T minComponent() const 
    {
        T m = (*this)[0];

        for(int i=1; i<3; i++)
        {
            if((*this)[i] < m)
                m = (*this)[i];
        }

        return m;
    }

    T maxComponent() const 
    {
        T m = (*this)[0];

        for(int i=1; i<3; i++)
        {
            if((*this)[i] > m)
                m = (*this)[i];
        }

        return m;
    }

    Vector3T<T> componentWiseMin(const Vector3T<T> &v2) const 
    {
        Vector3T<T> v(*this);
        v.minimize(v2);
        return v;
    }

    Vector3T<T> componentWiseMax(const Vector3T<T> &v2) const {
        Vector3T<T> v(*this);
        v.maximize(v2);
        return v;
    }

    void minimize(const Vector3T<T> &v2) 
    {
        for(int i=0; i<3; i++)
        {
            if(v2[i] < (*this)[i])
                (*this)[i] = v2[i];
        }
    }

    void maximize(const Vector3T<T> &v2) 
    {
        for(int i=0; i<3; i++)
        {
            if(v2[i] > (*this)[i])
                (*this)[i] = v2[i];
        }
    }

    Vector3T<T> reflectionAt( const Vector3T<T> &n ) const 
    {
        return *this-2*(n*(*this))*n;
    }
    
    const T *data() const { return m_xyz; }

    operator const T *() const { return data(); }

public:
    static int dim() { return 3; }

    static const Vector3T<T> &zero() 
    {
        static const Vector3T<T> zeroVec(0,0,0);
        return zeroVec;
    }

private:
    T m_xyz[3];
};

template<class T>
Vector3T<T> operator*(T const &s, Vector3T<T> const &v)
{
    return v*s;
}

typedef Vector3T<double> Vector3;


#endif
