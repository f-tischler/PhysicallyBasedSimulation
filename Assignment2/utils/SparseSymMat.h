/******************************************************************
*
* SparseSymMat.h
*
* Description: 
*
* Symmetric sparse matrix, using dynamic data structures to allow fill-ins. 
* Note: Only lower-triagonal elements of matrix stored and accessible.
*
* Physically-Based Simulation Proseminar WS 2015
*
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

#ifndef __SPARSESYMMAT_T_H__
#define __SPARSESYMMAT_T_H__

#include <map>
using std::map;

template<class T>
class SparseSymmetricMatrixT
{
public:
    SparseSymmetricMatrixT(int numRowsCols) 
    {
        m_numCols = numRowsCols;
        m_rowData.resize(numRowsCols);
    }

    SparseSymmetricMatrixT() 
    {
        m_numCols = 0;
    }

    void Clear() 
    {
        m_numCols = 0;
        m_rowData.clear();
    }

    virtual ~SparseSymmetricMatrixT() {}

    void ClearResize(int numRowsCols) 
    {
        Clear();
        m_numCols = numRowsCols;
        m_rowData.resize(numRowsCols);
    }

    void MultVector(const vector<T> &x, vector<T> &b) const 
    {
        for(int i=0; i<(int)b.size(); i++)
            b[i] = 0;

        int nrows = GetNumRows();

        for(int row=0; row<nrows; row++)
        {
            const map<int, T> &rowData = m_rowData[row];
            
            T rowSum = 0;
            
            for(typename map<int, T>::const_iterator iter = rowData.begin(); iter != rowData.end(); iter++)
            {
                int col = iter->first;
                
                T val = iter->second;
                
                rowSum += val * x[col];
                
                if(col < row)
                    b[col] += val * x[row];
            }
            
            b[row] += rowSum;
        }
    }

    /* Modifies matrix and vector b so that linear system 'A*x = b' will have solution 
       "value" at index "idx". */
    void FixSolution(std::vector<T> &b, int idx, T value) 
    {
        int n = (int)b.size();
        
        map<int, T> &rowData = m_rowData[idx];

        for(typename map<int, T>::iterator iter = rowData.begin(); iter != rowData.end(); iter++)
        {
            int col = iter->first;
            
            b[col] -= iter->second * value;
            
            if(col == idx)
                iter->second = 1;
            else
                iter->second = 0;
        }
        
        b[idx] = value;

        for(int i=idx+1; i<n; i++)
        {
            T oldValue = GetAt(i, idx);
            if(oldValue != 0)
            {
                b[i] -= oldValue * value;
                GetAt(i, idx) = 0;
            }
        }
    }
    
    const T &operator()(int row, int col) const 
    {
        return GetAt(row, col);
    }

    T &operator()(int row, int col) 
    {
        return GetAt(row, col);
    }

    const T &GetAt(int row, int col) const 
    {
        const map<int, T> &rowData = m_rowData[row];
        
        typename map<int, T>::const_iterator iter = rowData.find(col);
        if(iter == rowData.end())
            return 0;
        
        return iter->second;
    }

    T &GetAt(int row, int col) 
    {
        map<int, T> &rowData = m_rowData[row];
        
        typename map<int, T>::iterator iter = rowData.find(col);
        if(iter == rowData.end())
        {
            rowData[col] = 0;
            iter = rowData.find(col);
        }
        
        return iter->second;
    }
    
    int GetNumRows() const { return m_rowData.size(); }
    int GetNumCols() const { return m_numCols; }

private:
    int m_numCols;
    vector<map<int, T> > m_rowData;
};

typedef SparseSymmetricMatrixT<double> SparseSymmetricMatrix;

#endif
