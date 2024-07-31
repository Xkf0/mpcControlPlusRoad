#pragma once
#ifndef RAC_PARAMETER_READER_H_
#define RAC_PARAMETER_READER_H_
#include <stdio.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
namespace rac
{
    namespace param
    {
        bool Has(const std::string &key);
        void Set(const std::string &key, const std::string &value);
        bool Get(const std::string &key, std::string &value);
        void Load(const std::string &param_fname, bool clear = false);
    }
    class ParameterReader
    {
    public:
        ParameterReader();
        ParameterReader(const std::string &param_fname);
        ~ParameterReader();

        template <typename T>
        bool param(const std::string &param_name, T &value, const T &default_val) const
        {
            std::string val_str;
            if (param::Get(param_name, val_str))
            {
                value = toValue(val_str, default_val);
                return true;
            }
            value = default_val;
            return false;
        }

        template <typename T>
        bool getParam(const std::string &param_name, T &value) const
        {
            std::string val_str;
            if (param::Get(param_name, val_str))
            {
                value = toValue(val_str, T());
                return true;
            }
            return false;
        }

        template <typename T>
        T param(const std::string &param_name, T default_val) const
        {
            T param_val;
            param(param_name, param_val, default_val);
            return param_val;
        }

        template <typename T>
        void setParam(const std::string &key, T &value)
        {
            param::Set(key, toString(value));
        }

        bool hasParam(const std::string &key) const;

    private:
        template <typename T>
        std::string toString(const T &value) const
        {
            std::string str;
            std::ostringstream oss;
            oss << value;
            str = oss.str();
            return str;
        }
        int mystrcmp(const char *s1, char *s2)
        {
            int i = 0;
            while (*s1 || *s2)
            {
                if (*s1 > *s2)
                {
                    return 1;
                }
                else if (*s1 < *s2)
                {
                    return -1;
                }
                else
                {
                    s1++;
                    s2++;
                }
            }
        }

        template <typename T>
        T toValue(const std::string &val_str, const T &default_val) const
        {
            if (val_str.empty())
            { // 为空
                return default_val;
            }
            T value = default_val;
            std::string val_type = typeid(T).name();
            if (typeid(T) == typeid(bool))
            { // 布尔型
                std::string::size_type postion = val_str.find("true");
                value = postion == std::string::npos ? false : true;
            }
            else if (typeid(T) == typeid(int) || typeid(T) == typeid(float) || typeid(T) == typeid(double))
            { // 数字型
                int p_count = 0;
                for (unsigned int i = 0; i < val_str.size(); ++i)
                {
                    if (!isdigit(val_str[i]))
                    {
                        std::string temp_str = std::to_string(val_str[i]);
                        if (temp_str == "+" || temp_str == "-")
                        {
                            if (i != 0)
                            {
                                return value;
                            }
                        }
                        else if (temp_str == ".")
                        {
                            if (p_count = 0)
                            {
                                p_count++;
                            }
                            else
                            {
                                return value;
                            }
                        }
                    }
                }
                std::istringstream iss(val_str);
                iss >> value;
            }
            else if (typeid(T) == typeid(std::string) || typeid(T) == typeid(char))
            {
                std::istringstream iss(val_str);
                iss >> value;
            }
            return value;
        }
    };
} // namespace rac
#endif // RAC_PARAMETER_READER_H_