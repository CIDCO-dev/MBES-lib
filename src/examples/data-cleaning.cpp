#ifndef DATACLEANING_CPP
#define DATACLEANING_CPP

#include <cstdio>
#include <string>
#include <iostream>
#include <list>

class PointFilter{
   public:
	PointFilter(){

	}

	~PointFilter(){

	}

	//returns true if we remove this point
	virtual bool filterPoint(uint64_t microEpoch,double x,double y,double z, uint32_t quality,uint32_t intensity) = 0;
};

class QualityFilter : public PointFilter{
   public:

	QualityFilter(int minimumQuality) : minimumQuality(minimumQuality){

	}

	~QualityFilter(){

	}

	bool filterPoint(uint64_t microEpoch,double x,double y,double z, uint32_t quality,uint32_t intensity){
		return quality < minimumQuality;
	}

  private:
	unsigned int minimumQuality;

};


int main(int argc,char** argv){

	std::string line;

	//Filter chain
	std::list<PointFilter *> filters;

	//TODO: load desired filters and parameters from command line
        if (argc < 2)
        {
            filters.push_back(new QualityFilter(0));
            std::cout << "Quality filter 0 add" << std::endl;
        }
        else
        {
            int i = 1;
            int quality;
            while(i < argc)
            {
                
                if (std::sscanf(argv[i],"%d",&quality)==1)
                {
                    filters.push_back(new QualityFilter(quality));
                    std::cout << "Quality filter " << quality << " add" << std::endl;
                }
                else
                {
                    std::cerr << "Error: parameter " << argv[i] << " invalid" << std::endl;
                }
                i = i+1;
            }
        }
        unsigned int lineCount = 1;
        while((std::getline(std::cin,line))&&(line!="0")){
            uint64_t microEpoch;
            double x,y,z;
            uint32_t quality;
            uint32_t intensity;

            if(sscanf(line.c_str(),"%lu %lf %lf %lf %d %d",&microEpoch,&x,&y,&z,&quality,&intensity)==6){
		bool doFilter = false;

			//Apply filter chain
		for(auto i = filters.begin();i!= filters.end();i++){
                    if( (*i)->filterPoint(microEpoch,x,y,z,quality,intensity) ){
			doFilter = true;
                        break;
                    }
		}

		if(!doFilter){
                    printf("%lu %.6lf %.6lf %.6lf %d %d\r\n",microEpoch,x,y,z,quality,intensity);
		}
            }
            else{
		std::cerr << "Error at line " << lineCount << std::endl;
            }
                
            lineCount++;
        }
    }
#endif
