//
// Created by jzz on 23-2-8.
//

#ifndef HAND_EYE_CALIB_INIPARSER_H
#define HAND_EYE_CALIB_INIPARSER_H


#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <map>

static std::string &TrimString(std::string &str);

// INI节点结构体
class ININode {
public:
    ININode(std::string root, std::string key, std::string value);

    ~ININode();

    std::string root;
    std::string key;
    std::string value;
};

// 键值对结构体
class SubNode {
public:
    SubNode();

    ~SubNode();

    /*!
     * @brief 插入元素
     * @param key
     * @param value
     */
    void InsertElement(const std::string &key, const std::string &value);

    std::map<std::string, std::string> sub_node;
};

class INIParser {

public:
    INIParser();

    ~INIParser();

public:

    /*!
     * @brief 读取INI文件
     * @param path
     * @return
     */
    int ReadINI(const std::string& path);

    /*!
     * @brief 由根结点和键获取值
     * @param root
     * @param key
     * @return
     */
    std::string GetValue(const std::string& root, const std::string& key);

    /*!
     * @brief 获取INI文件的结点数
     * @return
     */
    std::vector<ININode>::size_type GetSize();

    /*!
     * @brief 设置根结点和键获取值
     * @param root
     * @param key
     * @param value
     * @return
     */
    std::vector<ININode>::size_type SetValue(const std::string& root,
                                             const std::string& key,
                                             const std::string& value);

    /*!
     * @brief 写入INI文件
     * @param path
     * @return
     */
    int WriteINI(const std::string& path);

    /*!
     * @brief 清空
     */
    void Clear();

    /*!
     * @brief 遍历打印INI文件
     */
    void Travel();

private:
    std::map<std::string, SubNode> map_ini;        //INI文件内容的存储变量

};


#endif //HAND_EYE_CALIB_INIPARSER_H
