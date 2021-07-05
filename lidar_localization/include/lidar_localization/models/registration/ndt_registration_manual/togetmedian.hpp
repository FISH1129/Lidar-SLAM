#ifndef CPU_MEDIAN_H_
#define CPU_MEDIAN_H_

#include <vector>
#include <iostream>
#include <algorithm>
#include <functional>

//using namespace std;

namespace lidar_localization {
class toGetMedian{
public:
    void Insert(double num)
    {
       int size = maxheap.size() + minheap.size();
       if(size&1){                                   //第偶数个的情况
            double tmp = num;
            if((!maxheap.empty())&&(num < maxheap[0])){
                tmp = maxheap[0];
                maxheap[0] = num;
                make_heap(maxheap.begin(),maxheap.end());
            }   

            minheap.push_back(tmp);
            push_heap(minheap.begin(),minheap.end(),std::greater<double>());
       }
       else{
            double tmp = num;
            if((!minheap.empty())&&(num > minheap[0])){
                tmp = minheap[0];
                minheap[0] = num;
                make_heap(minheap.begin(),minheap.end(),std::greater<double>());
            }   

            maxheap.push_back(tmp);
            push_heap(maxheap.begin(),maxheap.end());
       }
    }  

    double GetMedian()
    {
        int size = maxheap.size() + minheap.size();
        double resu;
        if(size & 1){
            resu = maxheap[0];
        }   
        else{
            resu =  (maxheap[0] + minheap[0])/2.;
        }   

        return resu;
    }   
private:
    std::vector<double> maxheap;
    std::vector<double> minheap;
}; 
}

#endif