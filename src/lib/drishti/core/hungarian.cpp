/*!
  @file   hungarian.cpp
  @brief  Adaptation of hungarian assignment algorithm from OpenCV:

  Lineage : opencv/sources/opencv/modules/shape/src/sc_dis.cpp:SCDMatcher::hungarian

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved (Modifications)
  \license{Modifications and original OpenCV source released under the 3 Clause BSD License.}
*/

#include "drishti/core/hungarian.h"

#include <opencv2/core/core.hpp>

#include <iostream>

DRISHTI_CORE_NAMESPACE_BEGIN

double hungarian(const cv::Mat &costMatrix, DMatchVec &outMatches, IntVec &inliers1, IntVec &inliers2, int sizeScd1, int sizeScd2)
{
    IntVec free(costMatrix.rows, 0), collist(costMatrix.rows, 0);
    IntVec matches(costMatrix.rows, 0), colsol(costMatrix.rows), rowsol(costMatrix.rows);
    FloatVec d(costMatrix.rows), pred(costMatrix.rows), v(costMatrix.rows);
    
    const float LOWV = 1e-10f;
    bool unassignedfound;
    int  i=0, imin=0, numfree=0, prvnumfree=0, f=0, i0=0, k=0, freerow=0;
    int  j=0, j1=0, j2=0, endofpath=0, last=0, low=0, up=0;
    float min=0, h=0, umin=0, usubmin=0, v2=0;
    
    // COLUMN REDUCTION //
    for (j = costMatrix.rows-1; j >= 0; j--)
    {
        // find minimum cost over rows.
        min = costMatrix.at<float>(0,j);
        imin = 0;
        for (i = 1; i < costMatrix.rows; i++)
        {
            if (costMatrix.at<float>(i,j) < min)
            {
                min = costMatrix.at<float>(i,j);
                imin = i;
            }
        }
        v[j] = min;
        
        if (++matches[imin] == 1)
        {
            rowsol[imin] = j;
            colsol[j] = imin;
        }
        else
        {
            colsol[j]=-1;
        }
    }
    
    // REDUCTION TRANSFER //
    for (i=0; i<costMatrix.rows; i++)
    {
        if (matches[i] == 0)
        {
            free[numfree++] = i;
        }
        else
        {
            if (matches[i] == 1)
            {
                j1=rowsol[i];
                min=std::numeric_limits<float>::max();
                for (j=0; j<costMatrix.rows; j++)
                {
                    if (j!=j1)
                    {
                        if (costMatrix.at<float>(i,j)-v[j] < min)
                        {
                            min=costMatrix.at<float>(i,j)-v[j];
                        }
                    }
                }
                v[j1] = v[j1]-min;
            }
        }
    }
    // AUGMENTING ROW REDUCTION //
    int loopcnt = 0;
    do
    {
        loopcnt++;
        k=0;
        prvnumfree=numfree;
        numfree=0;
        while (k < prvnumfree)
        {
            i=free[k];
            k++;
            umin = costMatrix.at<float>(i,0)-v[0];
            j1=0;
            usubmin = std::numeric_limits<float>::max();
            for (j=1; j<costMatrix.rows; j++)
            {
                h = costMatrix.at<float>(i,j)-v[j];
                if (h < usubmin)
                {
                    if (h >= umin)
                    {
                        usubmin = h;
                        j2 = j;
                    }
                    else
                    {
                        usubmin = umin;
                        umin = h;
                        j2 = j1;
                        j1 = j;
                    }
                }
            }
            i0 = colsol[j1];
            
            if (fabs(umin-usubmin) > LOWV) //if( umin < usubmin )
            {
                v[j1] = v[j1] - (usubmin - umin);
            }
            else // minimum and subminimum equal.
            {
                if (i0 >= 0) // minimum column j1 is assigned.
                {
                    j1 = j2;
                    i0 = colsol[j2];
                }
            }
            // (re-)assign i to j1, possibly de-assigning an i0.
            rowsol[i]=j1;
            colsol[j1]=i;
            
            if (i0 >= 0)
            {
                //if( umin < usubmin )
                if (fabs(umin-usubmin) > LOWV)
                {
                    free[--k] = i0;
                }
                else
                {
                    free[numfree++] = i0;
                }
            }
        }
    }while (loopcnt<2); // repeat once.
    
    // AUGMENT SOLUTION for each free row //
    for (f = 0; f<numfree; f++)
    {
        freerow = free[f];       // start row of augmenting path.
        // Dijkstra shortest path algorithm.
        // runs until unassigned column added to shortest path tree.
        for (j = 0; j < costMatrix.rows; j++)
        {
            d[j] = costMatrix.at<float>(freerow,j) - v[j];
            pred[j] = float(freerow);
            collist[j] = j;        // init column list.
        }
        
        low=0; // columns in 0..low-1 are ready, now none.
        up=0;  // columns in low..up-1 are to be scanned for current minimum, now none.
        unassignedfound = false;
        do
        {
            if (up == low)
            {
                last=low-1;
                min = d[collist[up++]];
                for (k = up; k < costMatrix.rows; k++)
                {
                    j = collist[k];
                    h = d[j];
                    if (h <= min)
                    {
                        if (h < min) // new minimum.
                        {
                            up = low; // restart list at index low.
                            min = h;
                        }
                        collist[k] = collist[up];
                        collist[up++] = j;
                    }
                }
                for (k=low; k<up; k++)
                {
                    if (colsol[collist[k]] < 0)
                    {
                        endofpath = collist[k];
                        unassignedfound = true;
                        break;
                    }
                }
            }
            
            if (!unassignedfound)
            {
                // update 'distances' between freerow and all unscanned columns, via next scanned column.
                j1 = collist[low];
                low++;
                i = colsol[j1];
                h = costMatrix.at<float>(i,j1)-v[j1]-min;
                
                for (k = up; k < costMatrix.rows; k++)
                {
                    j = collist[k];
                    v2 = costMatrix.at<float>(i,j) - v[j] - h;
                    if (v2 < d[j])
                    {
                        pred[j] = float(i);
                        if (v2 == min)
                        {
                            if (colsol[j] < 0)
                            {
                                // if unassigned, shortest augmenting path is complete.
                                endofpath = j;
                                unassignedfound = true;
                                break;
                            }
                            else
                            {
                                collist[k] = collist[up];
                                collist[up++] = j;
                            }
                        }
                        d[j] = v2;
                    }
                }
            }
        } while (!unassignedfound);
        
        // update column prices.
        for (k = 0; k <= last; k++)
        {
            j1 = collist[k];
            v[j1] = v[j1] + d[j1] - min;
        }
        
        // reset row and column assignments along the alternating path.
        do
        {
            i = int(pred[endofpath]);
            colsol[endofpath] = i;
            j1 = endofpath;
            endofpath = rowsol[i];
            rowsol[i] = j1;
        } while (i != freerow);
    }
    
    // calculate symmetric shape context cost
    cv::Mat trueCostMatrix(costMatrix, cv::Rect(0,0,sizeScd2, sizeScd1));
    float leftcost = 0;
    for (int nrow=0; nrow<trueCostMatrix.rows; nrow++)
    {
        double minval;
        minMaxIdx(trueCostMatrix.row(nrow), &minval);
        leftcost+=float(minval);
    }
    leftcost /= trueCostMatrix.rows;
    
    float rightcost = 0;
    for (int ncol=0; ncol<trueCostMatrix.cols; ncol++)
    {
        double minval;
        minMaxIdx(trueCostMatrix.col(ncol), &minval);
        rightcost+=float(minval);
    }
    rightcost /= trueCostMatrix.cols;
    
    double minMatchCost = std::max(leftcost,rightcost);
    
    // Save in a DMatch vector
    for (i=0;i<costMatrix.cols;i++)
    {
        cv::DMatch singleMatch(colsol[i],i,costMatrix.at<float>(colsol[i],i));//queryIdx,trainIdx,distance
        outMatches.push_back(singleMatch);
    }
    
    // Update inliers
    inliers1.reserve(sizeScd1);
    for (size_t kc = 0; kc<inliers1.size(); kc++)
    {
        if (rowsol[kc]<sizeScd1) // if a real match
        {
            inliers1[kc]=1;
        }
        else
        {
            inliers1[kc]=0;
        }
    }
    inliers2.reserve(sizeScd2);
    for (size_t kc = 0; kc<inliers2.size(); kc++)
    {
        if (colsol[kc]<sizeScd2) // if a real match
        {
            inliers2[kc]=1;
        }
        else
        {
            inliers2[kc]=0;
        }
    }
    
    return minMatchCost;
}

DRISHTI_CORE_NAMESPACE_END
