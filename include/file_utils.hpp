#ifndef FILE_UTILS_XYW_HPP
#define FILE_UTILS_XYW_HPP
#include <dirent.h>
#include <string>
#include <string.h>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include "time_utils.hpp"
using namespace std;

class YamlParser
{
public:
    string cameraDir;
    string imuFile;
    YamlParser(/* args */) = default;
    ~YamlParser(){};
/**
 * @brief 导入yaml，并赋值
 * 
 */
    void parseYamlFile()
    {
        YAML::Node config = YAML::LoadFile("../param/params.yaml");
        cameraDir = config["cameraDirectory"].as<string>();
        imuFile = config["ImuDataFile"].as<string>();
    }
};
/** @brief 遍历path下所有文件及文件夹，将文件名存入filenames
 *  @param path 需要遍历的路径
 *  @param filenames 存储所有的文件名
 */
inline bool getFileNames(string path, vector<std::string> &filenames)
{
    DIR *pDir;
    struct dirent *ptr;
    if (!(pDir = opendir(path.c_str())))
        return false;
    while ((ptr = readdir(pDir)) != 0)
    {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
            filenames.push_back(path + "/" + ptr->d_name);
    }
    closedir(pDir);
    return true;
}
/**
 * @brief 删除文件或文件夹
 * 
 * @param path 
 */
inline int removeFolder(string path){
    string command = "rm -r " + path;
    return system(command.c_str());
}
/**
 * @brief 判断是否是空文件夹
 * 
 * @param path 
 * @return true 
 * @return false 
 */
bool isEmptyFolder(string path){
    DIR *pDir;
    struct dirent *ptr;
    if(!(pDir = opendir(path.c_str()))){
        cout << path << " doesn't exist." << endl;
        return false;
    }
    while((ptr = readdir(pDir)) != 0){
        if( strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
            return false;
        }
    }
    
    return true;
}
/**
 * @brief 拼接dir和filename得到绝对路径
 * 
 */
inline string getAbsDir(string dir, string name)
{
    return dir + "/" + name;
}

/**
 * @brief 从字符串中提取所有的数字，存入vector,其中第一个是时间戳因此要特殊对待
 * 
 * @param str 
 * @return vector<float> 
 */
vector<double> extractDoubleFromString(string str)
{
    string numbers("0123456789.-");
    size_t inxStart = str.find_first_of(numbers);
    size_t offset = 0;
    vector<double> numberList;
    bool timeFlag = true;
    while (inxStart != string::npos)
    {
        if(timeFlag)
        {
            size_t inxEnd = str.find_first_not_of(numbers, inxStart);
            numberList.push_back(string2time(str.substr(inxStart,inxEnd-inxStart)));
            timeFlag = false;
            inxStart = str.find_first_of(numbers, inxEnd);
            continue;
        }
        // stod使用时必须使字符串起始就是数字
        double num = stod(str.substr(inxStart),&offset); // 执行后inxStart指向第一个未被转换的元素
        numberList.push_back(num);
        inxStart = str.find_first_of(numbers, inxStart+offset);
    }

    return numberList;
}
/**
 * @brief 若路径不存在则新建文件夹
 * 
 */
int createNewFolder(string path){
    DIR *pDir;
    struct dirent *ptr;
    if (!(pDir = opendir(path.c_str()))){
        string command = "mkdir -p " + path;
        return system(command.c_str());
    }
    return 0;
}


#endif