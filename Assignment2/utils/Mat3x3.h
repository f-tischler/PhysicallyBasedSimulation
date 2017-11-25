/******************************************************************
*
* Mat3x3.h
*
* Description: Code providing helper function for handling 3x3
* matrices; standard operators are provided
*
* Physically-Based Simulation Proseminar WS 2015
*
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/


#ifndef __MAT3x3_T_H__
#define __MAT3x3_T_H__

template<typename Scalar>
class Matrix3x3T
{
public:
    Matrix3x3T() {}

    Matrix3x3T(const Matrix3x3T &p) 
    {
        m_elem[0][0] = p.m_elem[0][0];
        m_elem[0][1] = p.m_elem[0][1];
        m_elem[0][2] = p.m_elem[0][2];
        m_elem[1][0] = p.m_elem[1][0];
        m_elem[1][1] = p.m_elem[1][1];
        m_elem[1][2] = p.m_elem[1][2];
        m_elem[2][0] = p.m_elem[2][0];
        m_elem[2][1] = p.m_elem[2][1];
        m_elem[2][2] = p.m_elem[2][2];
    }
    
    Scalar& operator() (int _i, int _j) 
    {
        return m_elem[_i][_j];
    }
    
    const Scalar& operator() (int _i, int _j) const 
    {
        return m_elem[_i][_j];
    }

    Matrix3x3T& operator+=( const Matrix3x3T& p ) 
    {
        m_elem[0][0] += p.m_elem[0][0];
        m_elem[0][1] += p.m_elem[0][1];
        m_elem[0][2] += p.m_elem[0][2];
        m_elem[1][0] += p.m_elem[1][0];
        m_elem[1][1] += p.m_elem[1][1];
        m_elem[1][2] += p.m_elem[1][2];
        m_elem[2][0] += p.m_elem[2][0];
        m_elem[2][1] += p.m_elem[2][1];
        m_elem[2][2] += p.m_elem[2][2];
        return *this;
    }

    Matrix3x3T& operator-=( const Matrix3x3T& p ) 
    {
        m_elem[0][0] -= p.m_elem[0][0];
        m_elem[0][1] -= p.m_elem[0][1];
        m_elem[0][2] -= p.m_elem[0][2];
        m_elem[1][0] -= p.m_elem[1][0];
        m_elem[1][1] -= p.m_elem[1][1];
        m_elem[1][2] -= p.m_elem[1][2];
        m_elem[2][0] -= p.m_elem[2][0];
        m_elem[2][1] -= p.m_elem[2][1];
        m_elem[2][2] -= p.m_elem[2][2];
        return *this;
    }
    
    Matrix3x3T& operator*=( Scalar s ) 
    {
        m_elem[0][0] *= s;
        m_elem[0][1] *= s;
        m_elem[0][2] *= s;
        m_elem[1][0] *= s;
        m_elem[1][1] *= s;
        m_elem[1][2] *= s;
        m_elem[2][0] *= s;
        m_elem[2][1] *= s;
        m_elem[2][2] *= s;
        return *this;
    }

    Matrix3x3T& operator*=( const Matrix3x3T& p ) 
    {
        return ( *this = *this * p );
    }

    Matrix3x3T operator+( const Matrix3x3T& p ) const 
    {
        return Matrix3x3T( 
            m_elem[0][0] + p.m_elem[0][0],
            m_elem[0][1] + p.m_elem[0][1],
            m_elem[0][2] + p.m_elem[0][2],
            m_elem[1][0] + p.m_elem[1][0],
            m_elem[1][1] + p.m_elem[1][1],
            m_elem[1][2] + p.m_elem[1][2],
            m_elem[2][0] + p.m_elem[2][0],
            m_elem[2][1] + p.m_elem[2][1],
            m_elem[2][2] + p.m_elem[2][2] );
    }

    Matrix3x3T operator-( const Matrix3x3T& p ) const 
    {
        return Matrix3x3T( 
            m_elem[0][0] - p.m_elem[0][0],
            m_elem[0][1] - p.m_elem[0][1],
            m_elem[0][2] - p.m_elem[0][2],
            m_elem[1][0] - p.m_elem[1][0],
            m_elem[1][1] - p.m_elem[1][1],
            m_elem[1][2] - p.m_elem[1][2],
            m_elem[2][0] - p.m_elem[2][0],
            m_elem[2][1] - p.m_elem[2][1],
            m_elem[2][2] - p.m_elem[2][2] );
    }

    Matrix3x3T operator/( Scalar s ) const 
    {
        return Matrix3x3T( 
            m_elem[0][0] / s, m_elem[0][1] / s, m_elem[0][2] / s,
            m_elem[1][0] / s, m_elem[1][1] / s, m_elem[1][2] / s,
            m_elem[2][0] / s, m_elem[2][1] / s, m_elem[2][2] / s );
    }

    Matrix3x3T operator*( Scalar s ) const 
    {
        return Matrix3x3T( 
            m_elem[0][0] * s, m_elem[0][1] * s, m_elem[0][2] * s,
            m_elem[1][0] * s, m_elem[1][1] * s, m_elem[1][2] * s,
            m_elem[2][0] * s, m_elem[2][1] * s, m_elem[2][2] * s );
    }

    friend Matrix3x3T operator*( Scalar s, const Matrix3x3T& p) 
    {
        return Matrix3x3T( 
            p.m_elem[0][0] * s, p.m_elem[0][1] * s, p.m_elem[0][2] * s,
            p.m_elem[1][0] * s, p.m_elem[1][1] * s, p.m_elem[1][2] * s,
            p.m_elem[2][0] * s, p.m_elem[2][1] * s, p.m_elem[2][2] * s );
    }

    Vector3T<Scalar> operator*( const Vector3T<Scalar>& vec ) const 
    {
        return Vector3T<Scalar>( 
            m_elem[0][0] * vec[0] +
            m_elem[0][1] * vec[1] +
            m_elem[0][2] * vec[2],
            m_elem[1][0] * vec[0] +
            m_elem[1][1] * vec[1] +
            m_elem[1][2] * vec[2],
            m_elem[2][0] * vec[0] +
            m_elem[2][1] * vec[1] +
            m_elem[2][2] * vec[2] );
    }

    Matrix3x3T operator*( const Matrix3x3T& p ) const 
    {
        Matrix3x3T result;
        result.m_elem[0][0] = (m_elem[0][0]*p.m_elem[0][0] +
                               m_elem[0][1]*p.m_elem[1][0] +
                               m_elem[0][2]*p.m_elem[2][0] );
        result.m_elem[0][1] = (m_elem[0][0]*p.m_elem[0][1] +
                               m_elem[0][1]*p.m_elem[1][1] +
                               m_elem[0][2]*p.m_elem[2][1] );
        result.m_elem[0][2] = (m_elem[0][0]*p.m_elem[0][2] +
                               m_elem[0][1]*p.m_elem[1][2] +
                               m_elem[0][2]*p.m_elem[2][2] );
        result.m_elem[1][0] = (m_elem[1][0]*p.m_elem[0][0] +
                               m_elem[1][1]*p.m_elem[1][0] +
                               m_elem[1][2]*p.m_elem[2][0] );
        result.m_elem[1][1] = (m_elem[1][0]*p.m_elem[0][1] +
                               m_elem[1][1]*p.m_elem[1][1] +
                               m_elem[1][2]*p.m_elem[2][1] );
        result.m_elem[1][2] = (m_elem[1][0]*p.m_elem[0][2] +
                               m_elem[1][1]*p.m_elem[1][2] +
                               m_elem[1][2]*p.m_elem[2][2] );
        result.m_elem[2][0] = (m_elem[2][0]*p.m_elem[0][0] +
                               m_elem[2][1]*p.m_elem[1][0] +
                               m_elem[2][2]*p.m_elem[2][0] );
        result.m_elem[2][1] = (m_elem[2][0]*p.m_elem[0][1] +
                               m_elem[2][1]*p.m_elem[1][1] +
                               m_elem[2][2]*p.m_elem[2][1] );
        result.m_elem[2][2] = (m_elem[2][0]*p.m_elem[0][2] +
                               m_elem[2][1]*p.m_elem[1][2] +
                               m_elem[2][2]*p.m_elem[2][2] );
        return result;
    }
    
    void Zero() 
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                m_elem[i][j] = (Scalar)0;
    }

    void Identity() 
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
                m_elem[i][j] = (Scalar)0;
            m_elem[i][i] = (Scalar)1;
        }
    }
    
    Scalar Det() const
    {
        return ( 
            (m_elem[0][1]*m_elem[1][2] - m_elem[0][2]*m_elem[1][1]) * m_elem[2][0] +
            (m_elem[0][2]*m_elem[1][0] - m_elem[0][0]*m_elem[1][2]) * m_elem[2][1] +
            (m_elem[0][0]*m_elem[1][1] - m_elem[0][1]*m_elem[1][0]) * m_elem[2][2] );
    }

    Scalar Trace() const 
    {
        return m_elem[0][0] + m_elem[1][1] + m_elem[2][2];
    }

    Matrix3x3T Transposed() const 
    {
        return( Matrix3x3T( m_elem[0][0], m_elem[1][0], m_elem[2][0],
                            m_elem[0][1], m_elem[1][1], m_elem[2][1],
                            m_elem[0][2], m_elem[1][2], m_elem[2][2] ) );
    }

    void Transpose() 
    {
        Scalar a;
        
        a = m_elem[1][0];
        m_elem[1][0] = m_elem[0][1];
        m_elem[0][1] = a;
        
        a = m_elem[2][0];
        m_elem[2][0] = m_elem[0][2];
        m_elem[0][2] = a;
        
        a = m_elem[2][1];
        m_elem[2][1] = m_elem[1][2];
        m_elem[1][2] = a;
    }
    
    Matrix3x3T<Scalar> Inverse() const 
    {
        Matrix3x3T<Scalar> M;
        ComputeInverseUnsave(M);
        return M;
    }

    Matrix3x3T<Scalar> Inverse(bool &ok) const 
    {
        Matrix3x3T<Scalar> M;       
        ok = ComputeInverse(M, std::numeric_limits<Scalar>::epsilon());
        
        return M;
    }
    
    bool Invert() 
    {
        bool ok = false;
        *this = Inverse(ok);
        return ok;
    }
    
    const Scalar *Data() const { return &m_elem[0][0]; }    
    operator const Scalar *() const { return Data(); }

private:
    bool ComputeInverse(Matrix3x3T<Scalar> &M, Scalar eps) const 
    {
        Scalar d = - m_elem[0][0]*m_elem[1][1]*m_elem[2][2]
                   + m_elem[0][0]*m_elem[1][2]*m_elem[2][1]
                   + m_elem[1][0]*m_elem[0][1]*m_elem[2][2]
                   - m_elem[1][0]*m_elem[0][2]*m_elem[2][1]
                   - m_elem[2][0]*m_elem[0][1]*m_elem[1][2]
                   + m_elem[2][0]*m_elem[0][2]*m_elem[1][1];
        
        if( fabs(d) <= eps )
        {
            M.setValues(std::numeric_limits<Scalar>::quiet_NaN());

            return false;
        }

        M(0,0) = (m_elem[1][2]*m_elem[2][1] - m_elem[1][1]*m_elem[2][2]) / d;
        M(0,1) = (m_elem[0][1]*m_elem[2][2] - m_elem[0][2]*m_elem[2][1]) / d;
        M(0,2) = (m_elem[0][2]*m_elem[1][1] - m_elem[0][1]*m_elem[1][2]) / d;
        M(1,0) = (m_elem[1][0]*m_elem[2][2] - m_elem[1][2]*m_elem[2][0]) / d;
        M(1,1) = (m_elem[0][2]*m_elem[2][0] - m_elem[0][0]*m_elem[2][2]) / d;
        M(1,2) = (m_elem[0][0]*m_elem[1][2] - m_elem[0][2]*m_elem[1][0]) / d;
        M(2,0) = (m_elem[1][1]*m_elem[2][0] - m_elem[1][0]*m_elem[2][1]) / d;
        M(2,1) = (m_elem[0][0]*m_elem[2][1] - m_elem[0][1]*m_elem[2][0]) / d;
        M(2,2) = (m_elem[0][1]*m_elem[1][0] - m_elem[0][0]*m_elem[1][1]) / d;

        return true;
    }

    void ComputeInverseUnsave(Matrix3x3T<Scalar> &M) const 
    {

        Scalar d =  - m_elem[0][0]*m_elem[1][1]*m_elem[2][2]
                    + m_elem[0][0]*m_elem[1][2]*m_elem[2][1]
                    + m_elem[1][0]*m_elem[0][1]*m_elem[2][2]
                    - m_elem[1][0]*m_elem[0][2]*m_elem[2][1]
                    - m_elem[2][0]*m_elem[0][1]*m_elem[1][2]
                    + m_elem[2][0]*m_elem[0][2]*m_elem[1][1];

        M(0,0) = (m_elem[1][2]*m_elem[2][1] - m_elem[1][1]*m_elem[2][2]) / d;
        M(0,1) = (m_elem[0][1]*m_elem[2][2] - m_elem[0][2]*m_elem[2][1]) / d;
        M(0,2) = (m_elem[0][2]*m_elem[1][1] - m_elem[0][1]*m_elem[1][2]) / d;
        M(1,0) = (m_elem[1][0]*m_elem[2][2] - m_elem[1][2]*m_elem[2][0]) / d;
        M(1,1) = (m_elem[0][2]*m_elem[2][0] - m_elem[0][0]*m_elem[2][2]) / d;
        M(1,2) = (m_elem[0][0]*m_elem[1][2] - m_elem[0][2]*m_elem[1][0]) / d;
        M(2,0) = (m_elem[1][1]*m_elem[2][0] - m_elem[1][0]*m_elem[2][1]) / d;
        M(2,1) = (m_elem[0][0]*m_elem[2][1] - m_elem[0][1]*m_elem[2][0]) / d;
        M(2,2) = (m_elem[0][1]*m_elem[1][0] - m_elem[0][0]*m_elem[1][1]) / d;
    }

private:
    Scalar m_elem[3][3];
};

typedef Matrix3x3T<double> Matrix3x3;

#endif
