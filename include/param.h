/*************************************************************************
	> File Name: include/param.h
	> Author: zqlee
	> Mail: zqlee@whu.edu.cn
	> Created Time: Jul.18, 2015
    > note：read parameters
 ************************************************************************/
#ifndef INCLUDE_PARAM_H_
#define INCLUDE_PARAM_H_

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

/*
If I devide the declaration and the definition of this class into two separate 
parts in this header file, I will be informed that "multi-definition" of the 
member function since this header file is included by several cpp source files.
To avoid this problem, put the declaration and definition into two fiels. 
*/

class ParameterReader
{
public:
    ParameterReader(std::string filename)
    {
        std::ifstream fin(filename);
        if (!fin)
        {
            //std::cerr<<"parameter file does not exist."<<std::endl;
            status_=0;
            return;
        }
        while(!fin.eof())
        {
            std::string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // comment begins with "#"
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            std::string key = str.substr( 0, pos );
            std::string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    std::string getData(std::string key )
    {
        std::map<std::string, std::string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            std::cout<<"Parameter name "<<key<<" not found!"<<std::endl;
            return std::string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    std::map<std::string, std::string> data;
    int status_=1;
};
#endif //INCLUDE_PARAM_H_
