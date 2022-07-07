#include "medianfilter.hpp"

int main() 

{

    MedianFilter filter(7, 6); 
    
    for (int i = 0; i < 7; ++i) 
    
    {

        for (int k = 0; k < 6; ++k) 
        
        {

            filter.setData(i*k, k);

        }

        

    }

    filter.computeMedian();
    std::cout << filter.getMedian();

    return 0; 

}