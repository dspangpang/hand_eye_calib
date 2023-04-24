//
// Created by jzz on 23-2-8.
//

#include <utility>

#include "../include/ini_parser/INIParser.h"


ININode::ININode(std::string root, std::string key, std::string value) {
    this->root = std::move(root);
    this->key = std::move(key);
    this->value = std::move(value);
}

ININode::~ININode() = default;

SubNode::SubNode() = default;

SubNode::~SubNode() = default;

void SubNode::InsertElement(const std::string& key, const std::string& value) {
    sub_node.insert(std::pair<std::string, std::string>(key, value));
}

INIParser::INIParser() = default;

INIParser::~INIParser() = default;

int INIParser::ReadINI(const std::string& path) {
    std::ifstream in_conf_file(path.c_str());
    if (!in_conf_file) return 0;
    std::string str_line;
    std::string str_root;
    std::vector<ININode> vec_ini;
    while (getline(in_conf_file, str_line))
    {
        std::string::size_type left_pos;
        std::string::size_type right_pos;
        std::string::size_type equal_div_pos;
        std::string str_key;
        std::string str_value;
        if ((std::string::npos != (left_pos = str_line.find('['))) &&
            (std::string::npos != (right_pos = str_line.find(']'))))
        {
            //std::cout << str_line.substr(left_pos+1, right_pos-1) << std::endl;
            str_root = str_line.substr(left_pos + 1, right_pos - 1);
        }

        if (std::string::npos != (equal_div_pos = str_line.find('=')))
        {
            str_key = str_line.substr(0, equal_div_pos);
            str_value = str_line.substr(equal_div_pos + 1, str_line.size() - 1);
            str_key = TrimString(str_key);
            str_value = TrimString(str_value);
            //cout << str_key << "=" << str_value << endl;
        }

        if ((!str_root.empty()) && (!str_key.empty()) && (!str_value.empty()))
        {
            ININode ini_node(str_root, str_key, str_value);
            vec_ini.push_back(ini_node);
            //std::cout << vec_ini.size() << std::endl;
        }
    }
    in_conf_file.close();
    in_conf_file.clear();

    //vector convert to map
    std::map<std::string, std::string> map_tmp;
    for (auto & itr : vec_ini)
    {
        map_tmp.insert(std::pair<std::string, std::string>(itr.root, ""));
    }	//提取出根节点
    for (auto & itr : map_tmp)
    {
#ifdef INIDEBUG
        cout << "根节点： " << itr->first << endl;
#endif	//INIDEBUG
        SubNode sn;
        for (auto & sub_itr : vec_ini)
        {
            if (sub_itr.root == itr.first)
            {
#ifdef INIDEBUG
                cout << "键值对： " << sub_itr->key << "=" << sub_itr->value << endl;
#endif	//INIDEBUG
                sn.InsertElement(sub_itr.key, sub_itr.value);
            }
        }
        map_ini.insert(std::pair<std::string, SubNode>(itr.first, sn));
    }
    return 1;

}

std::string INIParser::GetValue(const std::string& root, const std::string& key) {
    auto itr = map_ini.find(root);
    auto sub_itr = itr->second.sub_node.find(key);
    if (!(sub_itr->second).empty())
        return sub_itr->second;
    return "";

}

std::vector<ININode>::size_type INIParser::SetValue(const std::string& root,
                                                    const std::string& key,
                                                    const std::string& value) {

    auto itr = map_ini.find(root);	//查找
    if (map_ini.end() != itr)
    {
        //itr->second.sub_node.insert(pair<string, string>(key, value));
        itr->second.sub_node[key] = value;
    }	//根节点已经存在了，更新值
    else
    {
        SubNode sn;
        sn.InsertElement(key, value);
        map_ini.insert(std::pair<std::string, SubNode>(root, sn));
    }	//根节点不存在，添加值

    return map_ini.size();

}

int INIParser::WriteINI(const std::string& path) {
    std::ofstream out_conf_file(path.c_str());
    if (!out_conf_file)
        return -1;
    //cout << map_ini.size() << endl;
    for (auto & itr : map_ini)
    {
        //cout << itr->first << endl;
        out_conf_file << "[" << itr.first << "]" << std::endl;
        for (auto & sub_itr : itr.second.sub_node)
        {
            //cout << sub_itr->first << "=" << sub_itr->second << endl;
            out_conf_file << sub_itr.first << "=" << sub_itr.second << std::endl;
        }
    }

    out_conf_file.close();
    out_conf_file.clear();

    return 1;

}

void INIParser::Clear() { map_ini.clear(); }

void INIParser::Travel() {
    for (auto & itr : this->map_ini)
    {
        //root
        std::cout << "[" << itr.first << "]" << std::endl;
        for (auto & itr1 : itr.second.sub_node)
        {
            std::cout << "    " << itr1.first << " = " << itr1.second << std::endl;
        }
    }

}

std::vector<ININode>::size_type INIParser::GetSize() { return map_ini.size(); }

std::string &TrimString(std::string &str) {
    std::string::size_type pos;
    while (std::string::npos != (pos = str.find(' ')))
        str = str.replace(pos, pos + 1, "");
    return str;
}
